use crate::envelope_generator::EngineType;

pub static OP_MENU_ITEMS: [MenuItem<OpAction>; 2] = [
    MenuItem {
        name: "EG TYPE",
        selection: OpAction::EngineType,
    },
    MenuItem {
        name: "EXIT",
        selection: OpAction::Exit,
    },
];

pub enum OpAction {
    EngineType,
    Exit,
}

pub static ENGINE_TYPE_MENU_ITEMS: [MenuItem<Option<EngineType>>; 4] = [
    MenuItem {
        name: "ADSR",
        selection: Some(EngineType::ADSR),
    },
    MenuItem {
        name: "ADDSR",
        selection: Some(EngineType::ADDSR),
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
        selection: AdminAction::Exit,
    },
    MenuItem {
        name: "DIAGNOSE",
        selection: AdminAction::Diagnose,
    },
    MenuItem {
        name: "EXIT",
        selection: AdminAction::Exit,
    },
];

pub struct MenuItem<SelectionT> {
    pub name: &'static str,
    pub selection: SelectionT,
}

pub enum AdminAction {
    Diagnose,
    Exit,
}
