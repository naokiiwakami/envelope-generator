use crate::envelope_generator::EngineType;

pub static OP_MENU_ITEMS: [MenuItem<OpAction>; 2] = [
    MenuItem {
        name: "EG TYPE",
        selection: OpAction::EngineType,
    },
    MenuItem {
        name: "CANCEL",
        selection: OpAction::Cancel,
    },
];

pub enum OpAction {
    EngineType,
    Cancel,
}

/// EG Engine type.
/// IMPORTANT: The menu index must be the same as u8 representation of the
/// EngineType entry, otherwise ControlPanel misbehaves on switching engine types.
pub static ENGINE_TYPE_MENU_ITEMS: [MenuItem<Option<EngineType>>; 5] = [
    MenuItem {
        name: "PARA DECAYS",
        selection: Some(EngineType::ParaDecays),
    },
    MenuItem {
        name: "ADDSR",
        selection: Some(EngineType::Addsr),
    },
    MenuItem {
        name: "ADSR",
        selection: Some(EngineType::Adsr),
    },
    MenuItem {
        name: "LINEAR",
        selection: Some(EngineType::Linear),
    },
    MenuItem {
        name: "CANCEL",
        selection: None,
    },
];

pub static ADMIN_MENU_ITEMS: [MenuItem<AdminAction>; 3] = [
    MenuItem {
        name: "CALIBRATE",
        selection: AdminAction::Calibrate,
    },
    MenuItem {
        name: "DIAGNOSE",
        selection: AdminAction::Diagnose,
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
    Cancel,
}
