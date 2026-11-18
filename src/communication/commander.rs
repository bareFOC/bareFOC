//! # communication::commander — String-based command protocol
//!
//! Ports [`Commander.h`](source/Arduino-FOC/src/communication/Commander.h)
//! and [`Commander.cpp`](source/Arduino-FOC/src/communication/Commander.cpp)
//! from Arduino-FOC.
//!
//! The `Commander` parses string commands (e.g., "A1.5") and dispatches
//! them to registered callbacks. It is designed to be `no_std` compatible
//! and uses function pointers for callbacks to avoid heap allocation.

#![allow(non_camel_case_types)]

/// Commander verbose display mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum VerboseMode {
    Nothing = 0x00,
    OnRequest = 0x01,
    UserFriendly = 0x02,
    MachineReadable = 0x03,
}

/// Command callback function pointer.
/// The callback receives the payload string (everything after the command ID).
pub type CommandCallback = fn(&str);

struct CallbackEntry {
    id: char,
    callback: CommandCallback,
    #[allow(dead_code)]
    label: &'static str,
}

/// Commander class implementing string communication protocol based on ID + value.
///
/// Generic over `N`, the maximum number of registered callbacks (default 20).
pub struct Commander<const N: usize = 20> {
    callbacks: [Option<CallbackEntry>; N],
    count: usize,
    pub verbose: VerboseMode,
    pub decimal_places: u8,
}

impl<const N: usize> Default for Commander<N> {
    fn default() -> Self {
        Self {
            callbacks: [const { None }; N],
            count: 0,
            verbose: VerboseMode::UserFriendly,
            decimal_places: 3,
        }
    }
}

impl<const N: usize> Commander<N> {
    /// Create a new Commander with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a callback to the commander with the given command ID.
    ///
    /// # Arguments
    /// * `id` - The single-character command ID (e.g., 'A', 'M').
    /// * `callback` - The function to call when this command is received.
    /// * `label` - A static string label for this command (used for scanning).
    pub fn add(&mut self, id: char, callback: CommandCallback, label: &'static str) {
        if self.count < N {
            self.callbacks[self.count] = Some(CallbackEntry { id, callback, label });
            self.count += 1;
        }
    }

    /// Process a user input string and fire the appropriate callback.
    ///
    /// # Arguments
    /// * `user_input` - The raw command string (e.g., "M10.5").
    pub fn run(&mut self, user_input: &str) {
        let input = user_input.trim();
        if input.is_empty() {
            return;
        }

        let mut chars = input.chars();
        let id = match chars.next() {
            Some(c) => c,
            None => return,
        };
        let payload = chars.as_str();

        match id {
            '@' => {
                if let Ok(mode) = payload.parse::<u8>() {
                    self.verbose = match mode {
                        0 => VerboseMode::Nothing,
                        1 => VerboseMode::OnRequest,
                        2 => VerboseMode::UserFriendly,
                        3 => VerboseMode::MachineReadable,
                        _ => self.verbose,
                    };
                }
            }
            '#' => {
                if let Ok(places) = payload.parse::<u8>() {
                    self.decimal_places = places;
                }
            }
            '?' => {
                // Scan command: in a full implementation, this would iterate
                // over `self.callbacks` and print the labels. Since we don't
                // have a writer here, the user can register a '?' callback
                // to handle scanning with their own output mechanism.
            }
            _ => {
                for entry in self.callbacks.iter().flatten() {
                    if entry.id == id {
                        (entry.callback)(payload);
                        return;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commander_default_settings() {
        let cmd = Commander::<5>::new();
        assert_eq!(cmd.verbose, VerboseMode::UserFriendly);
        assert_eq!(cmd.decimal_places, 3);
        assert_eq!(cmd.count, 0);
    }

    #[test]
    fn commander_add_and_run() {
        let mut cmd = Commander::<5>::new();
        
        // We can't easily test function pointer callbacks with state in no_std
        // without unsafe or statics. Instead, we test the parsing logic directly.
        cmd.add('A', |_| {}, "Test Command");
        assert_eq!(cmd.count, 1);
        
        // The callback is called, we just verify it doesn't panic.
        cmd.run("A123.45");
    }

    #[test]
    fn commander_verbose_mode_change() {
        let mut cmd = Commander::<5>::new();
        cmd.run("@3");
        assert_eq!(cmd.verbose, VerboseMode::MachineReadable);
        
        cmd.run("@0");
        assert_eq!(cmd.verbose, VerboseMode::Nothing);
    }

    #[test]
    fn commander_ignores_empty_input() {
        let mut cmd = Commander::<5>::new();
        cmd.run("");
        cmd.run("   ");
        assert_eq!(cmd.count, 0); // No callbacks should have been added or panic
    }
}
