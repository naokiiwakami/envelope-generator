pub static MENU_ITEMS: [MenuItem; 2] = [
    MenuItem {
        name: "MODE",
        action: Action::ChangeMode,
    },
    MenuItem {
        name: "EXIT",
        action: Action::Exit,
    },
];

pub struct MenuItem {
    pub name: &'static str,
    pub action: Action,
}

pub enum Action {
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

pub struct AdminMenuItem {
    pub name: &'static str,
    pub action: AdminAction,
}

pub enum AdminAction {
    Diagnose,
    Exit,
}
