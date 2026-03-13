pub enum Action {
    Diagnose,
    Exit,
}

pub struct MenuItem {
    pub name: &'static str,
    pub action: Action,
}

pub static ADMIN_MENU_ITEMS: [MenuItem; 2] = [
    MenuItem {
        name: "Diagnose",
        action: Action::Diagnose,
    },
    MenuItem {
        name: "Exit Menu",
        action: Action::Exit,
    },
];
