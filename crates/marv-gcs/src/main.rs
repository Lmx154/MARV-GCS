use marv_gcs::cli::{self, Parsed};

fn main() {
    match cli::parse(std::env::args().skip(1)) {
        Ok(Parsed::Help) => println!("{}", cli::USAGE),
        Ok(Parsed::Run(cli)) => std::process::exit(marv_gcs::run(cli)),
        Err(message) => {
            eprintln!("marv-gcs: {message}\n{}", cli::USAGE);
            std::process::exit(2);
        }
    }
}
