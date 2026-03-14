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
