#![allow(non_upper_case_globals)]

use keyberon::action::{
    d, k, l,
    Action::{self, *},
};
use usbd_human_interface_device::page::Keyboard::{self, *};

pub enum CustomAction {
    PrevTrack,
    NextTrack,
    Bootldr,
}

const PREVTRACK: Action<CustomAction, Keyboard> = Custom(CustomAction::PrevTrack);
const NEXTTRACK: Action<CustomAction, Keyboard> = Custom(CustomAction::NextTrack);
const BOOTLDR: Action<CustomAction, Keyboard> = Custom(CustomAction::Bootldr);

/// Allows to rename enum variant, eg to shorter names for better readability of the layout arrays.
macro_rules! rename_keys {
    ($($new:ident : $old:ident)+) => {
        $(
            const $new: Keyboard = Keyboard::$old;
         )+
    };
}
rename_keys! {
    Slash  : ForwardSlash
    PgUp   : PageUp
    PgDwn  : PageDown
    LShift : LeftShift
    LCtrl  : LeftControl
    LAlt   : LeftAlt
    LGUI   : LeftGUI
    LBrace : LeftBrace
    RBrace : RightBrace
    RAlt   : RightAlt
    Kb1    : Keyboard1
    Kb2    : Keyboard2
    Kb3    : Keyboard3
    Kb4    : Keyboard4
    Kb5    : Keyboard5
    Kb6    : Keyboard6
    Kb7    : Keyboard7
    Kb8    : Keyboard8
    Kb9    : Keyboard9
    Kb0    : Keyboard0
    PrntScr: PrintScreen
    Enter  : ReturnEnter
    Del    : DeleteForward
    BkSpc  : DeleteBackspace
    LArrow : LeftArrow
    RArrow : RightArrow
    UArrow : UpArrow
    DArrow : DownArrow
    Kp1    : Keypad1
    Kp2    : Keypad2
    Kp3    : Keypad3
    Kp4    : Keypad4
    Kp5    : Keypad5
    Kp6    : Keypad6
    Kp7    : Keypad7
    Kp8    : Keypad8
    Kp9    : Keypad9
    Kp0    : Keypad0
    KpDot  : KeypadDot
    KpNLock: KeypadNumLockAndClear
    KpDiv  : KeypadDivide
    KpMul  : KeypadMultiply
    KpSub  : KeypadSubtract
    KpAdd  : KeypadAdd
    KpEnter: KeypadEnter
    SColon : Semicolon
    Esc    : Escape
}

/// ```text
/// . . . . . .         . . . . . .
/// . . . . . . °     ° . . . . . .
/// . . . . . . °     ° . . . . . .
/// . . . . . .  .   .  . . . . . .
/// . . . .  ,  . . . .  ,  . . . .
///              .   .
/// ```

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<14, 5, 4, CustomAction, Keyboard> = [
    [
        [k(Grave),  k(Kb1),     k(Kb2), k(Kb3),  k(Kb4), k(Kb5),   NoOp,      NoOp,     k(Kb6),   k(Kb7),    k(Kb8),    k(Kb9),    k(Kb0),    k(LBrace)],
        [k(Tab),    k(Q),       k(W),   k(E),    k(R),   k(T),     NoOp,      k(Equal), k(Y),     k(U),      k(I),      k(O),      k(P),      k(RBrace)],
        [k(LShift), k(A),       k(S),   k(D),    k(F),   k(G),     d(3),      l(2),     k(H),     k(J),      k(K),      k(L),      k(SColon), k(Apostrophe)],
        [k(LCtrl),  k(Z),       k(X),   k(C),    k(V),   k(B),     k(Insert), k(Del),   k(N),     k(M),      k(Comma),  k(Dot),    k(Slash),  k(NonUSHash)],
        [k(Pause),  k(PrntScr), NoOp,   k(LAlt), k(Esc), k(Enter), k(Space),  k(BkSpc), l(1),     k(RAlt),   k(LGUI),   k(Minus),  NoOp,      k(ScrollLock)],
    ], [
        [NoOp,      k(F1),      k(F2),  k(F3),   k(F4),  k(F5),    NoOp,      NoOp,     NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp],
        [NoOp,      k(F6),      k(F7),  k(F9),   k(F9),  k(F10),   NoOp,      NoOp,     k(PgUp),  k(Home),   k(UArrow), k(End),    NoOp,      NoOp],
        [Trans,     k(F11),     k(F12), NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     k(PgDwn), k(LArrow), k(DArrow), k(RArrow), NoOp,      NoOp],
        [Trans,     NoOp,       NoOp,   Trans,   NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp],
        [NoOp,      NoOp,       NoOp,   Trans,   NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     Trans,     Trans,     NoOp,      NoOp,      NoOp]
    ], [
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     PREVTRACK, NoOp,      NEXTTRACK, NoOp,      NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp],
    ], [
        [BOOTLDR,   NoOp,       NoOp,   NoOp,    NoOp,   NoOp,     NoOp,      NoOp,     NoOp,     k(KpNLock),k(KpDiv),  k(KpMul),  k(KpSub),  NoOp],
        [NoOp,      k(Q),       k(W),   k(E),    k(R),   k(T),     NoOp,      NoOp,     NoOp,     k(Kp7),    k(Kp8),    k(Kp9),    k(KpAdd),  NoOp],
        [k(LShift), k(A),       k(S),   k(D),    k(F),   NoOp,     d(0),      NoOp,     NoOp,     k(Kp4),    k(Kp5),    k(Kp6),    k(KpAdd),  NoOp],
        [k(LCtrl),  k(Z),       k(X),   k(C),    k(V),   NoOp,     k(Insert), k(Del),   NoOp,     k(Kp1),    k(Kp2),    k(Kp3),    k(KpEnter),NoOp],
        [k(Pause),  k(PrntScr), NoOp,   k(LAlt), k(Esc), k(Enter), k(Space),  k(BkSpc), NoOp,     k(Kp0),    k(Kp0),    k(KpDot),  k(KpEnter),NoOp]
    ]
];
