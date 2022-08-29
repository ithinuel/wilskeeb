#[no_mangle]
pub extern "aapcs" fn __aeabi_lmul(a: u64, b: u64) -> u64 {
    compiler_builtins::int::mul::__aeabi_lmul::__aeabi_lmul(a, b)
}

#[no_mangle]
pub unsafe extern "aapcs" fn __aeabi_memmove8(dest: *mut u8, src: *const u8, n: usize) {
    compiler_builtins::mem::memmove(dest, src, n);
}

#[no_mangle]
pub unsafe extern "C" fn memcmp(s1: *const u8, s2: *const u8, n: usize) -> i32 {
    compiler_builtins::mem::memcmp(s1, s2, n)
}
