use crate::envelope_generator::EngineType;

/// EG Engine type.
/// IMPORTANT: The menu index must be the same as u8 representation of the
/// EngineType entry, otherwise ControlPanel misbehaves on switching engine types.
pub const ENGINE_TYPE_MENU_ITEMS: [MenuItem<Option<EngineType>>; 4] = [
    MenuItem {
        name: "PARA DECAYS",
        selection: Some(EngineType::ParaDecays),
    },
    MenuItem {
        name: "ADDSR",
        selection: Some(EngineType::TwoDecays),
    },
    MenuItem {
        name: "ADSR",
        selection: Some(EngineType::Adsr),
    },
    MenuItem {
        name: "LINEAR",
        selection: Some(EngineType::Linear),
    },
    /*
    MenuItem {
        name: "CANCEL",
        selection: None,
    },
    */
];

pub const POLARITY_CHANGE_TARGET_ITEMS: [MenuItem<u8>; 3] = [
    MenuItem {
        name: "BOTH",
        selection: 3,
    },
    MenuItem {
        name: "VOICE_1",
        selection: 1,
    },
    MenuItem {
        name: "VOICE_2",
        selection: 2,
    },
];

pub const ADMIN_MENU_ITEMS: [MenuItem<AdminAction>; 4] = [
    MenuItem {
        name: "CALIBRATE",
        selection: AdminAction::Calibrate,
    },
    MenuItem {
        name: "DIAGNOSE",
        selection: AdminAction::Diagnose,
    },
    MenuItem {
        name: "TOGGLE DIAG",
        selection: AdminAction::ToggleDiagnoseMode,
    },
    MenuItem {
        name: "CANCEL",
        selection: AdminAction::Cancel,
    },
];

pub struct MenuItem<SelectionT> {
    pub name: &'static str,
    pub selection: SelectionT,
}

pub enum AdminAction {
    Calibrate,
    Diagnose,
    ToggleDiagnoseMode,
    Cancel,
}
