use core::cell::RefCell;

pub struct StateInner {}
pub struct State(RefCell<StateInner>);

impl State {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self(RefCell::new(StateInner {}))
    }
}
impl keyberon::keyboard::Leds for &State {
    fn caps_lock(&mut self, _status: bool) {}

    fn num_lock(&mut self, _status: bool) {}

    fn scroll_lock(&mut self, _status: bool) {}

    fn compose(&mut self, _status: bool) {}

    fn kana(&mut self, _status: bool) {}
}
