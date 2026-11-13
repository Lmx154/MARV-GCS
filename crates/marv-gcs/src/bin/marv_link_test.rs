use marv_gcs::link_test::{self, Parsed};

fn main() {
    match link_test::parse(std::env::args().skip(1)) {
        Ok(Parsed::Help) => println!("{}", link_test::USAGE),
        Ok(Parsed::Run(args)) => match link_test::run(&args, &mut std::io::stdout()) {
            Ok(report) if report.link_error.is_none() => {}
            Ok(_) => std::process::exit(1),
            Err(message) => {
                eprintln!("marv-link-test: {message}");
                std::process::exit(1);
            }
        },
        Err(message) => {
            eprintln!("marv-link-test: {message}\n{}", link_test::USAGE);
            std::process::exit(2);
        }
    }
}
