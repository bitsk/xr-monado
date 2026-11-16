// SPDX-License-Identifier: GPL-3.0-or-later
use std::{env, path::PathBuf, process::Command};
fn main() {
    for name in ["passthrough.comp", "background.vert", "background.frag"] {
        println!("cargo:rerun-if-changed=shaders/{name}");
        let status =
            Command::new(env::var_os("GLSLANG_VALIDATOR").unwrap_or("glslangValidator".into()))
                .args(["-V", "--target-env", "vulkan1.0", "-o"])
                .arg(PathBuf::from(env::var_os("OUT_DIR").unwrap()).join(format!("{name}.spv")))
                .arg(format!("shaders/{name}"))
                .status()
                .expect("run glslangValidator");
        assert!(status.success(), "compile {name}");
    }
    println!("cargo:rerun-if-env-changed=GLSLANG_VALIDATOR");
}
