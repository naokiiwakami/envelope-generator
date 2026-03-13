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
        name: "Calibrate",
        action: AdminAction::Exit,
    },
    MenuItem {
        name: "Diagnose",
        action: AdminAction::Diagnose,
    },
    MenuItem {
        name: "Exit",
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
