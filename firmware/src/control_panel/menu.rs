use crate::envelope_generator::EngineType;

pub static OP_MENU_ITEMS: [MenuItem<OpAction>; 2] = [
    MenuItem {
        name: "MODE",
        action: OpAction::ChangeMode,
    },
    MenuItem {
        name: "EXIT",
        action: OpAction::Exit,
    },
];

pub enum OpAction {
    ChangeMode,
    Exit,
}

pub static MODE_MENU_ITEMS: [MenuItem<EngineType>; 2] = [
    MenuItem {
        name: "ADSR",
        action: EngineType::Default,
    },
    MenuItem {
        name: "ADDSR",
        action: EngineType::ADDSR,
    },
];

pub static ADMIN_MENU_ITEMS: [MenuItem<AdminAction>; 3] = [
    MenuItem {
        name: "CALIBRATE",
        action: AdminAction::Exit,
    },
    MenuItem {
        name: "DIAGNOSE",
        action: AdminAction::Diagnose,
    },
    MenuItem {
        name: "EXIT",
        action: AdminAction::Exit,
    },
];

pub struct MenuItem<ActionT> {
    pub name: &'static str,
    pub action: ActionT,
}

pub enum AdminAction {
    Diagnose,
    Exit,
}
