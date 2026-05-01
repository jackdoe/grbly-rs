use crate::ui::{console, controls, editor, scene};

#[derive(Default)]
pub struct UiState {
    pub controls: controls::ControlsState,
    pub editor: editor::EditorState,
    pub console: console::ConsoleState,
    pub material: scene::MaterialState,
    pub material_version: u32,
    pub theme_set: bool,
}
