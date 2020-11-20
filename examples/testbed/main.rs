mod main_loop;
mod clipboard;
mod settings;
mod test;
mod draw;
mod draw_private;
mod test_private;
mod tests;

fn main() {
    let system = main_loop::init(file!());
    system.main_loop();
}

