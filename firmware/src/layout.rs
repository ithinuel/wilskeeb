use keyberon::{
    action::{
        d, k, l,
        Action::{self, *},
    },
    key_code::KeyCode::*,
};

pub enum CustomAction {
    PrevTrack,
    NextTrack,
    Bootldr,
}

const PREVTRACK: Action<CustomAction> = Action::Custom(CustomAction::PrevTrack);
const NEXTTRACK: Action<CustomAction> = Action::Custom(CustomAction::NextTrack);
const BOOTLDR: Action<CustomAction> = Action::Custom(CustomAction::Bootldr);

/// ```text
/// . . . . . .         . . . . . .
/// . . . . . . °     ° . . . . . .
/// . . . . . . °     ° . . . . . .
/// . . . . . .  .   .  . . . . . .
/// . . . .  ,  . . . .  ,  . . . .
///              .   .
/// ```

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<14, 5, 4, CustomAction> = [
    [
        [k(Grave),  k(Kb1),     k(Kb2), k(Kb3),  k(Kb4),    k(Kb5),   NoOp,      NoOp,      k(Kb6),    k(Kb7),    k(Kb8),   k(Kb9),      k(Kb0),      k(LBracket)],
        [k(Tab),    k(Q),       k(W),   k(E),    k(R),      k(T),     NoOp,      NoOp,      k(Y),      k(U),      k(I),     k(O),        k(P),        k(RBracket)],
        [k(LShift), k(A),       k(S),   k(D),    k(F),      k(G),     d(3),      l(2),      k(H),      k(J),      k(K),     k(L),        k(SColon),   k(Quote)],
        [k(LCtrl),  k(Z),       k(X),   k(C),    k(V),      k(B),     k(Insert), k(Delete), k(N),      k(M),      k(Comma), k(Dot),      k(Slash),    k(NonUsHash)],
        [k(Pause),  k(PScreen), NoOp,   k(LAlt), k(Escape), k(Enter), k(Space),  k(BSpace), l(1),      k(RAlt),   k(LGui),  k(Minus),    k(Equal),    k(ScrollLock)],
    ], [
        [NoOp,      k(F1),      k(F2),  k(F3),   k(F4),     k(F5),    NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
        [NoOp,      k(F6),      k(F7),  k(F9),   k(F9),     k(F10),   NoOp,      NoOp,      k(PgUp),   k(Home),   k(Up),    k(End),      NoOp,        NoOp],
        [Trans,     k(F11),     k(F12), NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      k(PgDown), k(Left),   k(Down),  k(Right),    NoOp,        NoOp],
        [Trans,     NoOp,       NoOp,   Trans,   NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   Trans,   NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      Trans,     Trans,    NoOp,        NoOp,        NoOp]
    ], [
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      PREVTRACK, NoOp,     NEXTTRACK,   NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
    ], [
        [BOOTLDR,   NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      NoOp,     NoOp,        NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      k(Kp7),    k(Kp8),   k(Kp9),      NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     d(0),      NoOp,      NoOp,      k(Kp4),    k(Kp5),   k(Kp6),      NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      k(Kp1),    k(Kp2),   k(Kp3),      NoOp,        NoOp],
        [NoOp,      NoOp,       NoOp,   NoOp,    NoOp,      NoOp,     NoOp,      NoOp,      NoOp,      NoOp,      k(Kp0),   k(KpDot),    NoOp,        NoOp]
    ]
];
