//! A double-buffered sparse grid keyed by A5 cell.
//!
//! This mirrors krABMaga's `SparseA5Grid` contract (spec §4.2): agents read the
//! committed bags and write to a separate side, and nothing an agent writes is
//! visible until [`SparseCellGrid::lazy_update`] commits it. Building the server
//! on this rather than directly on krABMaga is what lets the whole stack
//! compile, test and run without Bevy; the krABMaga-backed grid is a drop-in
//! behind the `krabmaga` feature.

use std::collections::HashMap;

use sim_core::{AgentId, GeoCell};

/// Occupancy of record for a run.
#[derive(Debug, Clone)]
pub struct SparseCellGrid {
    /// Committed side. Agents read here.
    read_cells: HashMap<GeoCell, Vec<AgentId>>,
    read_agents: HashMap<AgentId, GeoCell>,
    /// In-flight side. Agents and intents write here.
    write_cells: HashMap<GeoCell, Vec<AgentId>>,
    write_agents: HashMap<AgentId, GeoCell>,
    resolution: i32,
    root: Option<GeoCell>,
}

impl SparseCellGrid {
    pub fn new(resolution: i32, root: Option<GeoCell>) -> Self {
        SparseCellGrid {
            read_cells: HashMap::new(),
            read_agents: HashMap::new(),
            write_cells: HashMap::new(),
            write_agents: HashMap::new(),
            resolution,
            root,
        }
    }

    pub fn resolution(&self) -> i32 {
        self.resolution
    }

    pub fn root(&self) -> Option<GeoCell> {
        self.root
    }

    /// Whether a cell is addressable in this run: right resolution, and inside
    /// the island root when there is one.
    pub fn accepts(&self, cell: GeoCell) -> bool {
        if cell.resolution() != self.resolution {
            return false;
        }
        match self.root {
            Some(root) => cell.is_within(root),
            None => true,
        }
    }

    /// Where an agent is, on the committed side.
    pub fn location(&self, agent: AgentId) -> Option<GeoCell> {
        self.read_agents.get(&agent).copied()
    }

    /// Who is in a cell, on the committed side.
    pub fn occupants(&self, cell: GeoCell) -> &[AgentId] {
        self.read_cells.get(&cell).map_or(&[], Vec::as_slice)
    }

    pub fn agent_count(&self) -> u64 {
        self.read_agents.len() as u64
    }

    pub fn occupied_cells(&self) -> u64 {
        self.read_cells.len() as u64
    }

    /// Committed cells, in ascending cell order. Ascending A5 index is also
    /// hierarchical order, so parents group their children together.
    pub fn sorted_cells(&self) -> Vec<GeoCell> {
        let mut cells: Vec<GeoCell> = self.read_cells.keys().copied().collect();
        cells.sort_unstable();
        cells
    }

    /// Place or move an agent on the write side.
    ///
    /// This is the `move_object` helper the spec asks krABMaga for (§5): the
    /// agent-to-cell and cell-to-agents indices are always updated together, so
    /// they cannot drift apart.
    pub fn set_location(&mut self, agent: AgentId, cell: GeoCell) {
        if let Some(previous) = self.write_agents.insert(agent, cell) {
            if previous == cell {
                return;
            }
            detach(&mut self.write_cells, previous, agent);
        }
        self.write_cells.entry(cell).or_default().push(agent);
    }

    /// Remove an agent from the write side. Returns where it was.
    pub fn remove(&mut self, agent: AgentId) -> Option<GeoCell> {
        let cell = self.write_agents.remove(&agent)?;
        detach(&mut self.write_cells, cell, agent);
        Some(cell)
    }

    /// Where an agent will be once the current tick commits.
    pub fn pending_location(&self, agent: AgentId) -> Option<GeoCell> {
        self.write_agents.get(&agent).copied()
    }

    /// Commit the write side and prepare the next one.
    ///
    /// The next write side starts as a copy of what was just committed, so an
    /// agent that does not move stays put. `clone_from` reuses the existing
    /// allocations, which is what keeps this off the tick budget.
    pub fn lazy_update(&mut self) {
        std::mem::swap(&mut self.read_cells, &mut self.write_cells);
        std::mem::swap(&mut self.read_agents, &mut self.write_agents);
        self.write_cells.clone_from(&self.read_cells);
        self.write_agents.clone_from(&self.read_agents);
    }

    /// Drop everything on both sides.
    pub fn clear(&mut self) {
        self.read_cells.clear();
        self.read_agents.clear();
        self.write_cells.clear();
        self.write_agents.clear();
    }

    /// Reconfigure the address space. Implies [`SparseCellGrid::clear`],
    /// because cells at the old resolution do not mean anything at the new one.
    pub fn reconfigure(&mut self, resolution: i32, root: Option<GeoCell>) {
        self.resolution = resolution;
        self.root = root;
        self.clear();
    }

    /// Commit whatever is on the write side immediately, without a step.
    /// Used when seeding a scenario or restoring a checkpoint.
    pub fn commit_now(&mut self) {
        self.read_cells.clone_from(&self.write_cells);
        self.read_agents.clone_from(&self.write_agents);
    }
}

fn detach(cells: &mut HashMap<GeoCell, Vec<AgentId>>, cell: GeoCell, agent: AgentId) {
    if let Some(bag) = cells.get_mut(&cell) {
        if let Some(index) = bag.iter().position(|a| *a == agent) {
            bag.swap_remove(index);
        }
        if bag.is_empty() {
            cells.remove(&cell);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn london(resolution: i32) -> GeoCell {
        GeoCell::from_lonlat(-0.1276, 51.5072, resolution).unwrap()
    }

    #[test]
    fn writes_are_invisible_until_the_tick_commits() {
        let mut grid = SparseCellGrid::new(9, None);
        let cell = london(9);
        grid.set_location(1, cell);

        assert_eq!(grid.location(1), None, "read side is still last tick");
        assert_eq!(grid.pending_location(1), Some(cell));

        grid.lazy_update();
        assert_eq!(grid.location(1), Some(cell));
        assert_eq!(grid.occupants(cell), &[1]);
    }

    #[test]
    fn an_agent_that_does_not_move_stays_put() {
        let mut grid = SparseCellGrid::new(9, None);
        grid.set_location(1, london(9));
        grid.lazy_update();
        grid.lazy_update();
        assert_eq!(grid.location(1), Some(london(9)));
        assert_eq!(grid.agent_count(), 1);
    }

    #[test]
    fn moving_updates_both_indices_together() {
        let mut grid = SparseCellGrid::new(9, None);
        let from = london(9);
        let to = from.neighbors().unwrap()[0];
        grid.set_location(1, from);
        grid.set_location(2, from);
        grid.lazy_update();

        grid.set_location(1, to);
        grid.lazy_update();

        assert_eq!(grid.location(1), Some(to));
        assert_eq!(grid.occupants(from), &[2]);
        assert_eq!(grid.occupants(to), &[1]);
        assert_eq!(grid.occupied_cells(), 2);
    }

    #[test]
    fn emptying_a_cell_drops_it_from_the_occupancy() {
        let mut grid = SparseCellGrid::new(9, None);
        let cell = london(9);
        grid.set_location(1, cell);
        grid.lazy_update();

        assert_eq!(grid.remove(1), Some(cell));
        grid.lazy_update();
        assert_eq!(grid.occupied_cells(), 0);
        assert_eq!(grid.agent_count(), 0);
        assert!(grid.occupants(cell).is_empty());
    }

    #[test]
    fn an_island_run_refuses_cells_outside_its_root() {
        let root = london(3);
        let grid = SparseCellGrid::new(9, Some(root));
        assert!(grid.accepts(london(9)));
        assert!(!grid.accepts(london(8)), "wrong resolution");
        assert!(!grid.accepts(GeoCell::from_lonlat(151.2093, -33.8688, 9).unwrap()));
    }

    #[test]
    fn cells_come_back_in_hierarchical_order() {
        let mut grid = SparseCellGrid::new(9, None);
        for (id, cell) in london(9).grid_disk(2).unwrap().into_iter().enumerate() {
            grid.set_location(id as u64, cell);
        }
        grid.lazy_update();
        let cells = grid.sorted_cells();
        assert!(cells.windows(2).all(|w| w[0] < w[1]));
    }
}
