use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::serialize_b2_world::*;

use bincode;
use serde_json;
use serde_yaml;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

pub(crate) fn test_deserialize<D: UserDataType>(test_name: &str) -> B2worldPtr<D> {
	let test_dir = Path::new("serialize_test").join(Path::new(test_name));
	return world_from_json(&test_dir.join(Path::new("world.json.json")));
}

pub(crate) fn test_serialize_deserialize<D: UserDataType>(test_name: &str, world: B2worldPtr<D>) {
	let test_dir = Path::new("serialize_test").join(Path::new(test_name));
	std::fs::create_dir_all(test_dir.clone()).unwrap();

	world_to_json(world.clone(), &test_dir.join(Path::new("world.json")));
	let deserialized_world = world_from_json::<D>(&test_dir.join(Path::new("world.json")));
	world_to_json(
		deserialized_world.clone(),
		&test_dir.join(Path::new("world.json.json")),
	);

	world_to_yaml(
		deserialized_world.clone(),
		&test_dir.join(Path::new("world.yaml")),
	);
	let deserialized_world = world_from_yaml::<D>(&test_dir.join(Path::new("world.yaml")));
	world_to_json(
		deserialized_world.clone(),
		&test_dir.join(Path::new("world.yaml.json")),
	);

	world_to_bincode(
		deserialized_world.clone(),
		&test_dir.join(Path::new("world.bincode")),
	);
	let deserialized_world = world_from_bincode::<D>(&test_dir.join(Path::new("world.bincode")));
	world_to_json(
		deserialized_world.clone(),
		&test_dir.join(Path::new("world.bincode.json")),
	);
}

fn world_to_json<D: UserDataType>(world: B2worldPtr<D>, file_name: &Path) {
	let json_file = File::create(file_name).unwrap();
	serde_json::to_writer_pretty(json_file, &*world).expect("erro while writing yaml");
}

fn world_from_json<D: UserDataType>(file_name: &Path) -> B2worldPtr<D> {
	let file = File::open(file_name).unwrap();
	let reader = BufReader::new(file);
	return serde_json::from_reader::<_, B2worldDeserializeResult<D>>(reader)
		.unwrap()
		.world;
}

fn world_to_yaml<D: UserDataType>(world: B2worldPtr<D>, file_name: &Path) {
	let json_file = File::create(file_name).unwrap();
	serde_yaml::to_writer(json_file, &*world).expect("erro while writing yaml");
}

fn world_from_yaml<D: UserDataType>(file_name: &Path) -> B2worldPtr<D> {
	let file = File::open(file_name).unwrap();
	let reader = BufReader::new(file);
	return serde_yaml::from_reader::<_, B2worldDeserializeResult<D>>(reader)
		.unwrap()
		.world;
}

fn world_to_bincode<D: UserDataType>(world: B2worldPtr<D>, file_name: &Path) {
	let json_file = File::create(file_name).unwrap();
	bincode::serialize_into(json_file, &*world).expect("erro while writing yaml");
}

fn world_from_bincode<D: UserDataType>(file_name: &Path) -> B2worldPtr<D> {
	let file = File::open(file_name).unwrap();
	let reader = BufReader::new(file);
	return bincode::deserialize_from::<_, B2worldDeserializeResult<D>>(reader)
		.unwrap()
		.world;
}
