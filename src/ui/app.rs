use crate::ui::{console, controls, editor, probe};

#[derive(Default)]
pub struct UiState {
    pub controls: controls::ControlsState,
    pub editor: editor::EditorState,
    pub console: console::ConsoleState,
    pub probe: probe::ProbeState,
    pub theme_set: bool,
}
