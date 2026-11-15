//! The dependency rule: no GUI crate anywhere in the CLI's tree.

use std::process::Command;

#[test]
fn the_cli_and_gcs_link_depend_on_no_gui_crate() {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    for package in ["marv-cli", "gcs-link"] {
        let output = Command::new(&cargo)
            .args(["tree", "-p", package, "-e", "normal", "--offline"])
            .current_dir(env!("CARGO_MANIFEST_DIR"))
            .output()
            .expect("run cargo tree");
        assert!(
            output.status.success(),
            "cargo tree -p {package}: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        let tree = String::from_utf8_lossy(&output.stdout).to_lowercase();
        let gui: Vec<&str> = tree
            .lines()
            .filter(|l| {
                ["eframe", "egui", "winit", "wgpu", "glow"]
                    .iter()
                    .any(|g| l.contains(g))
            })
            .collect();
        assert!(tree.contains("gcs-mavlink"), "the tree looks empty: {tree}");
        assert!(gui.is_empty(), "{package} depends on a GUI crate: {gui:?}");
    }
}
