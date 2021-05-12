
// NOTE use for the GenerationalIndex?
// https://kyren.github.io/2018/09/14/rustconf-talk.html
// We want to access an array without reusing indices (uniqueness) 
//  - Each index is used exactly once per generation
// We want to make sure that we can easily find free entries in an array
//  - Normally, you find free entries by iterating through a potentially
//  large array
//  - We can now free entries and increment it's index generation

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub struct GenerationalIndex {
    index: usize,
    generation: usize,
}


/// Metadata for an active or inactive GenerationalIndex
pub struct AllocatorEntry {
    is_live: bool,
    generation: usize,
}


/// Manages and produces GenerationalIndices
pub struct GenerationalIndexAllocator {
    entries: Vec<AllocatorEntry>,
    free: Vec<usize>,
}

impl GenerationalIndexAllocator {

    fn new() -> Self {

        Self {
            entries: Vec::new(),
            free: Vec::new(),
        }

    }


    /// Allocates new GenerationalIndex and creates metadata AllocatorEntry to track if index is
    /// live and it's generation
    fn allocate(&mut self) -> GenerationalIndex {

        match self.free.pop() { // Remove last element of free index values
            Some(index) => { // if value popped, increment generation and reactivate AllocatorEntry

                self.entries[index].generation += 1;
                self.entries[index].is_live = true;

                // Reuse a free index, but at incremented generation
                GenerationalIndex {
                    index,
                    generation: self.entries[index].generation,
                }

            }
            None => { // No free values, push new live AllocatorEntry at generation 0

                self.entries.push(AllocatorEntry {
                    is_live: true,
                    generation: 0,
                });

                // At generation 0, take the index mapping to the new AllocatorEntry
                GenerationalIndex {
                    index: self.entries.len() - 1,
                    generation: 0,
                }

            }
        }

    }

    /// Deallocate AllocatorEntry for a live GenerationalIndex
    fn deallocate(&mut self, index: GenerationalIndex) -> bool {

        if self.is_live(index) { // if GenerationalIndex is live

            // Deactivate the AllocatorEntry for this Index
            self.entries[index.index].is_live = false;
            // Push the value for this Index to the free vector, so the next time an AllocatorEntry
            // is needed, it will be an Index with this index value but with a incremented generation
            self.free.push(index.index);

            true

        } else {

            false

        }
    }


    /// Checks if GenerationalIndex is active
    fn is_live(&self, index: GenerationalIndex) -> bool {

        index.index < self.entries.len() // index is a possible value
            && self.entries[index.index].generation == index.generation // GenerationalIndex generation matches it's AllocatorEntry's generation
            && self.entries[index.index].is_live // AllocatorEntry for this GenerationalIndex is live
    }

}



/// Generic container element tracking generation of stored data
pub struct ArrayEntry<T> {
    value: T,
    generation: usize,
}

/// std Vector implementing generic generational container elements
pub struct GenerationalIndexArray<T>(Vec<Option<ArrayEntry<T>>>);

impl<T> GenerationalIndexArray<T> {

    fn new() -> Self {

        Self(Vec::new())

    }


    /// Emplace new object at given GenerationalIndex and wrap with a corresponding ArrayEntry
    fn set(&mut self, index: GenerationalIndex, value: T) {

        // Fill the Container with None up to length of the index
        while self.0.len() <= index.index {
            self.0.push(None); // increments length by 1
        }

        // Check generation of ArrayEntry currently stored at index
        let prev_gen = match &self.0[index.index] {
            Some(entry) => entry.generation, // This index was previously used
            None => 0, // First time using this index
        };

        // Check if using a stale index
        if prev_gen > index.generation {
            panic!("Attempted to write to GenerationalIndexArray with an index from previous generation");
        }

        // Generate ArrayEntry using this GenerationalIndex
        self.0[index.index] = Some(ArrayEntry {
            value,
            generation: index.generation,
        });

    }


    /// Remove ArrayEntry at a given GenerationalIndex
    fn remove(&mut self, index: GenerationalIndex) {

        if index.index < self.0.len() {

            self.0[index.index] = None;

        }

    }


    /// Returns Optional borrow to object at GenerationalIndex
    fn get(&self, index: GenerationalIndex) -> Option<&T> {

        // If out-of-bounds, return None
        if index.index >= self.0.len() {

            return None;

        }

        // Check for ArrayEntry at Index
        match &self.0[index.index] {

            // Return ArrayEntry value if generation of index is the same as that storing the data
            // else return None
            Some(entry) => if entry.generation == index.generation {
                Some(&entry.value)
            } else {
                None
            },
            None => None,

        }

    }


    /// Returns Optional mutable borrow to object at GenerationalIndex
    fn get_mut(&mut self, index: GenerationalIndex) -> Option<&mut T> {

        if index.index >= self.0.len() {
            return None;
        }

        match &mut self.0[index.index] {

            Some(entry) => if entry.generation == index.generation {
                Some(&mut entry.value)
            } else {
                None
            },
            None => None,

        }

    }


    /// Return vector of GenerationalIndices 
    fn get_all_valid_indices(
        &self,
        allocator: &GenerationalIndexAllocator ) -> Vec<GenerationalIndex> {

        let mut result = Vec::new();

        for i in 0..self.0.len() {

            // If some entry exists
            if let Some(entry) = &self.0[i] {

                // Create tmp index for this entry
                let index = GenerationalIndex {
                    index: i,
                    generation: entry.generation,
                };

                // Check if this GenerationalIndex is live, push Index if it is
                if allocator.is_live(index) {
                    result.push(index);
                }
            }

        }

        result

    }


    /// Return first valid AllocatorEntry
    fn get_first_valid_entry(
        &self,
        allocator: &GenerationalIndexAllocator) -> Option<(GenerationalIndex, &T)> {

        for i in 0..self.0.len() {

            if let Some(entry) = &self.0[i] {
                let index = GenerationalIndex {
                    index: i,
                    generation: entry.generation,
                };
                if allocator.is_live(index) {
                    return Some((index, &entry.value));
                }
            }

        }

        None

    }
}



#[cfg(test)]

#[test]
fn test_GenerationalIndexAllocator_allocate() {

    let mut allocator = GenerationalIndexAllocator::new();

    let _ = allocator.allocate();
    let _ = allocator.allocate();

    assert_eq!(allocator.entries.len(), 2);
}


#[test]
fn test_GenerationalIndexAllocator_deallocate() {

    let mut allocator = GenerationalIndexAllocator::new();

    let index1 = allocator.allocate();
    let _ = allocator.allocate();
    allocator.deallocate(index1);

    assert_eq!(allocator.free.len(), 1);

}


#[test]
fn test_GenerationalIndexAllocator_is_live() {

    let mut allocator = GenerationalIndexAllocator::new();
    let index1 = allocator.allocate();

    assert_eq!(allocator.is_live(index1), true);

}


#[test]
fn test_GenerationalIndexArray_set() {

    let mut storage = GenerationalIndexArray::<(i32, i32, i32)>::new();
    let mut allocator = GenerationalIndexAllocator::new();
    let index = allocator.allocate();

    let data = (0, 1, 2);

    storage.set(index, data);

    assert_eq!(storage.0.len(), 1);

}


#[test]
fn test_GenerationalIndexArray_remove() {

    let mut storage = GenerationalIndexArray::<(i32, i32, i32)>::new();
    let mut allocator = GenerationalIndexAllocator::new();
    let index1 = allocator.allocate();
    let index2 = allocator.allocate();
    let index3 = allocator.allocate();

    let data1 = (0, 1, 2);
    let data2 = (3, 4, 5);
    let data3 = (6, 7, 8);

    storage.set(index1, data1);
    storage.set(index2, data2);
    storage.set(index3, data3);

    storage.remove(index1);

    assert_eq!(storage.get(index1), None);

}


#[test]
fn test_GenerationalIndexArray_get_all_valid_indices() {

    let mut storage = GenerationalIndexArray::<(i32, i32, i32)>::new();
    let mut allocator = GenerationalIndexAllocator::new();
    let index1 = allocator.allocate();
    let index2 = allocator.allocate();
    let index3 = allocator.allocate();

    let data1 = (0, 1, 2);
    let data2 = (3, 4, 5);
    let data3 = (6, 7, 8);

    storage.set(index1, data1);
    storage.set(index2, data2);
    storage.set(index3, data3);

    storage.remove(index1);

    let valid = storage.get_all_valid_indices(&allocator);

    assert_eq!(valid.len(), 2);

}


#[test]
fn test_GenerationalIndexArray_get_first_valid_index() {

    let mut storage = GenerationalIndexArray::<(i32, i32, i32)>::new();
    let mut allocator = GenerationalIndexAllocator::new();
    let index1 = allocator.allocate();
    let index2 = allocator.allocate();
    let index3 = allocator.allocate();

    let data1 = (0, 1, 2);
    let data2 = (3, 4, 5);
    let data3 = (6, 7, 8);

    storage.set(index1, data1);
    storage.set(index2, data2);
    storage.set(index3, data3);

    storage.remove(index1);

    if let Some(valid) = storage.get_first_valid_entry(&allocator) {

        assert_eq!(valid.0.index, 1);
        assert_eq!(valid.1, &(3i32,4i32,5i32));

    }

}
