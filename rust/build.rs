use cmake::Config;

#[cfg(feature = "tauri-sidecar")]
fn copy_libs(install_dir: &std::path::Path) {
  use std::fs;

  #[cfg(target_os = "macos")]
  const LIB_PATTERN: &str = "lib/lib*.dylib";

  #[cfg(target_os = "linux")]
  const LIB_PATTERN: &str = "lib/lib*.so";

  #[cfg(target_os = "windows")]
  const LIB_PATTERN: &str = "bin/*.dll";

  let native_lib_search_dir = install_dir.join(LIB_PATTERN);

  let native_lib_out_dir = install_dir.join("tauri-sidecar");

  let target_triple = std::env::var("TARGET").unwrap();

  fs::create_dir_all(&native_lib_out_dir).expect("Unable to create directory for native libraries");

  for entry in glob::glob(native_lib_search_dir.to_str().unwrap()).expect("Failed to find trajoptlib dylibs") {
    let file = entry.expect("error loading dylib");
    let output_file = format!("{}/{}-{}",
        native_lib_out_dir.display(), file.file_name().unwrap().to_str().unwrap(), target_triple);
    fs::copy(file.clone(), output_file).expect("Error copying dylib");
  }
}

fn main() -> miette::Result<()> {

  let mut cmake_config = Config::new("..");

  cmake_config.profile("RelWithDebInfo")
      .define("OPTIMIZER_BACKEND", "casadi");

  if cfg!(target_os = "windows") {
    cmake_config.generator("MinGW Makefiles")
        .define("CMAKE_CXX_COMPILER", "x86_64-w64-mingw32-g++")
        .define("CMAKE_C_COMPILER", "x86_64-w64-mingw32-gcc")
        .define("CMAKE_SHARED_LINKER_FLAGS", "-static-libgcc -static-libstdc++")
        .define("CMAKE_EXE_LINKER_FLAGS", "-static-libgcc -static-libstdc++");
  }

  if cfg!(target_os = "linux") {
    cmake_config
        .define("CMAKE_CXX_COMPILER", "g++")
        .define("CMAKE_C_COMPILER", "gcc");
  }

  let dst = cmake_config.build();

  println!("cargo:rustc-link-search=native={}/bin", dst.display());
  println!("cargo:rustc-link-search=native={}/lib", dst.display());
  println!("cargo:rustc-link-lib=TrajoptLib");

  cxx_build::bridge("src/lib.rs")  // returns a cc::Build
        .file("src/trajoptlib.cc")
        .include("include")
        .include(format!("{}/include", dst.display()))
        .flag_if_supported("-std=c++20")
        .compile("trajoptlib-rust");

  println!("cargo:rerun-if-changed=include/trajoptlib.h");
  println!("cargo:rerun-if-changed=src/trajoptlib.cc");
  println!("cargo:rerun-if-changed=src/lib.rs");


  #[cfg(feature = "tauri-sidecar")]
  copy_libs(dst.as_path());

  Ok(())
}
