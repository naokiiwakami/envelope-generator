pub static OP_MENU_ITEMS: [OpMenuItem; 2] = [
    OpMenuItem {
        name: "MODE",
        action: OpAction::ChangeMode,
    },
    OpMenuItem {
        name: "EXIT",
        action: OpAction::Exit,
    },
];

pub struct OpMenuItem {
    pub name: &'static str,
    pub action: OpAction,
}

pub enum OpAction {
    ChangeMode,
    Exit,
}

pub static ADMIN_MENU_ITEMS: [AdminMenuItem; 2] = [
    AdminMenuItem {
        name: "DIAGNOSE",
        action: AdminAction::Diagnose,
    },
    AdminMenuItem {
        name: "EXIT",
        action: AdminAction::Exit,
    },
];

pub static A_MENU_ITEMS: [MenuItem<AdminAction>; 2] = [
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

pub struct AdminMenuItem {
    pub name: &'static str,
    pub action: AdminAction,
}

pub enum AdminAction {
    Diagnose,
    Exit,
}
