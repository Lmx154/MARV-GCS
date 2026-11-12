fn main() {
    for p in gcs_transport::list_serial_ports() {
        match p.usb {
            Some(u) => println!(
                "{}  {:04x}:{:04x}  manufacturer={}  product={}  serial={}",
                p.path,
                u.vid,
                u.pid,
                u.manufacturer.as_deref().unwrap_or("-"),
                u.product.as_deref().unwrap_or("-"),
                u.serial_number.as_deref().unwrap_or("-"),
            ),
            None => println!("{}  (not USB)", p.path),
        }
    }
}
