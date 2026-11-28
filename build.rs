use std::env;
use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};

fn emit_check_cfg() {
    let cfgs = [
        "mcu_stm32h503cb",
        "mcu_stm32g431cb",
        "has_bootloader",
        "bus_i3c",
        "bus_i2c",
        "bus_uart",
        "bus_can",
        "drv_drv8411a",
        "drv_drv8316",
        "drv_drv8311",
        "sensor_tmag5273",
        "sensor_mt6701",
    ];
    for cfg in cfgs {
        println!("cargo:rustc-check-cfg=cfg({cfg})");
    }
}

fn main() {
    emit_check_cfg();

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR not set"));
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));

    let board_env = env::var("BAREFOC_BOARD")
        .unwrap_or_else(|_| "examples/microspora/barefoc.toml".to_string());
    let board_path = if Path::new(&board_env).is_absolute() {
        PathBuf::from(&board_env)
    } else {
        manifest_dir.join(&board_env)
    };

    println!("cargo:rerun-if-env-changed=BAREFOC_BOARD");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_BOOTLOADER");
    println!("cargo:rerun-if-changed={}", board_path.display());

    let toml_text = fs::read_to_string(&board_path)
        .unwrap_or_else(|e| panic!("Failed to read BAREFOC_BOARD at {}: {e}", board_path.display()));
    let mut board = bareconfig::bsp::schema::parse_barefoc_toml(&toml_text)
        .unwrap_or_else(|e| panic!("Invalid barefoc.toml at {}: {e}", board_path.display()));

    let bootloader_feature = env::var("CARGO_FEATURE_BOOTLOADER").is_ok();
    if bootloader_feature || board.bootloader.enabled {
        println!("cargo:rustc-cfg=has_bootloader");
    } else {
        // Standalone build without bootloader feature: start app at flash_origin
        board.memory.app_origin = board.memory.flash_origin;
    }

    let memory_x = bareconfig::bsp::codegen::generate_memory_x(&board);
    fs::write(out_dir.join("memory.x"), memory_x).expect("Failed to write memory.x");

    let bsp_code = bareconfig::bsp::codegen::generate_bsp_rust_code(&board);
    fs::write(out_dir.join("bsp_generated.rs"), bsp_code)
        .expect("Failed to write bsp_generated.rs");

    let defmt_x = out_dir.join("defmt.x");
    if !defmt_x.exists() {
        let _ = File::create(defmt_x).and_then(|mut f| f.write_all(b"SECTIONS { }"));
    }

    // Emit MCU capability cfg
    let mcu = board.board.mcu.to_ascii_lowercase();
    if mcu.starts_with("stm32h503") {
        println!("cargo:rustc-cfg=mcu_stm32h503cb");
    } else if mcu.starts_with("stm32g431") {
        println!("cargo:rustc-cfg=mcu_stm32g431cb");
    }

    // Emit primary communication bus capability cfg
    let primary_proto = board
        .primary_host_port()
        .map(|(_, p)| p.protocol.to_ascii_lowercase())
        .unwrap_or_else(|| "uart".to_string());
    match primary_proto.as_str() {
        "i3c" => println!("cargo:rustc-cfg=bus_i3c"),
        "i2c" => println!("cargo:rustc-cfg=bus_i2c"),
        "uart" => println!("cargo:rustc-cfg=bus_uart"),
        "can" | "fdcan" => println!("cargo:rustc-cfg=bus_can"),
        _ => {}
    }

    // Emit gate driver capability cfg
    match board.gate_driver.driver_type.to_ascii_lowercase().as_str() {
        "drv8411a" => println!("cargo:rustc-cfg=drv_drv8411a"),
        "drv8316" => println!("cargo:rustc-cfg=drv_drv8316"),
        "drv8311" => println!("cargo:rustc-cfg=drv_drv8311"),
        _ => {}
    }

    // Emit sensor capability cfg
    let sensor = board
        .primary_sensor()
        .map(|(_, s)| s.active_type().to_ascii_lowercase())
        .unwrap_or_else(|| "virtual".to_string());
    if sensor.starts_with("tmag5273") {
        println!("cargo:rustc-cfg=sensor_tmag5273");
    } else if sensor.starts_with("mt6701") {
        println!("cargo:rustc-cfg=sensor_mt6701");
    }

    println!("cargo:rustc-link-search={}", out_dir.display());
}
