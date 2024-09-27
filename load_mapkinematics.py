import argparse
import sys
import os
import pickle
import traceback
import h5py
from multiprocessing import Pool

import numpy as np
from tqdm import tqdm

from create_route import create_route
import load_dataset

from osm_wrapper import OSMWrapper
from utils import global_to_vehicle_coordinates


def retrieve_kinemetic_data_and_gt(scene_data):
    out_dict = {}
    lcm_data = scene_data["lcm_data"]
    kinematic_keys = [
        "lcm_egomotion_timestamp",
        "oxts_heading",
        "oxts_lat",
        "oxts_lon",
        "lcm_lat_acceleration",
        "lcm_lat_velocity",
        "lcm_lon_acceleration",
        "lcm_lon_velocity",
        "lcm_yaw_rate",
    ]
    ground_truth_keys = [key for key in lcm_data.keys() if key in ["oxts_lat", "oxts_lon", "lcm_egomotion_timestamp"]]
    quality_keys = [key for key in lcm_data.keys() if key.startswith("lcm") and "quality" in key]

    quality_dict = {key: value for key, value in lcm_data.items() if key in quality_keys}
    if not all([min(value) == 3 for value in quality_dict.values()]):
        raise ValueError("Data not valid")  # lcm data not valid

    if min(lcm_data["oxts_valid"]) != 1:
        raise ValueError("Data not valid")  # oxts data not valid

    len_oxts = len(lcm_data["oxts_heading"])
    middle_frame = int(len_oxts / 2)
    logs_per_second = 50
    step_size = 5
    # future_horizon = 5  # seconds
    observation_window = 3  # seconds
    start_index = int(middle_frame - observation_window * logs_per_second)
    # end_index = int(middle_frame + future_horizon * logs_per_second + step_size)

    out_dict["pred_time"] = {
        "lat": lcm_data["oxts_lat"][middle_frame],
        "lon": lcm_data["oxts_lon"][middle_frame],
        "heading": lcm_data["oxts_heading"][middle_frame],
    }
    kinematic_data = {
        key: value[start_index : middle_frame + 1 : step_size]
        for key, value in lcm_data.items()
        if key in kinematic_keys
    }
    kinematic_data["relative_time"] = [
        i - kinematic_data["lcm_egomotion_timestamp"][-1] for i in kinematic_data["lcm_egomotion_timestamp"]
    ]
    kinematic_data.pop("lcm_egomotion_timestamp")
    ground_truth_data = {key: value[middle_frame:] for key, value in lcm_data.items() if key in ground_truth_keys}
    ground_truth_data["relative_time"] = [
        i - ground_truth_data["lcm_egomotion_timestamp"][0] for i in ground_truth_data["lcm_egomotion_timestamp"]
    ]
    ground_truth_data.pop("lcm_egomotion_timestamp")

    out_dict["kinematics"] = kinematic_data
    out_dict["gt"] = ground_truth_data

    meta_keys = [
        "FC_ant_tlc_data_image_raw_path_kw",
        "frame_timestamp_date",
        "sequence_id",
        "suite_id",
        "vehicle",
        "route",
    ]
    for key in meta_keys:
        out_dict[key] = scene_data[key]

    # MAKE THINGS RELATIVE TO THE EGO VEHICLE
    relative_gt = global_to_vehicle_coordinates(
        np.vstack([out_dict["gt"]["oxts_lat"], out_dict["gt"]["oxts_lon"]]).T,
        [out_dict["pred_time"]["lat"], out_dict["pred_time"]["lon"]],
        out_dict["pred_time"]["heading"],
    )
    out_dict["gt"]["local_lat"] = relative_gt[:, 0].tolist()
    out_dict["gt"]["local_lon"] = relative_gt[:, 1].tolist()
    out_dict["gt"].pop("oxts_lat")
    out_dict["gt"].pop("oxts_lon")

    kin = np.vstack([out_dict["kinematics"]["oxts_lat"], out_dict["kinematics"]["oxts_lon"]]).T
    # kin = [[out_dict["kinematics"]["oxts_lat"][i], out_dict["kinematics"]["oxts_lon"][i]] for i in range(len(out_dict["kinematics"]["oxts_lat"]))]
    kin_local_coords = global_to_vehicle_coordinates(
        kin,
        [out_dict["pred_time"]["lat"], out_dict["pred_time"]["lon"]],
        out_dict["pred_time"]["heading"],
    )
    out_dict["kinematics"]["oxts_lat"] = kin_local_coords[:, 0].tolist()
    out_dict["kinematics"]["oxts_lon"] = kin_local_coords[:, 1].tolist()

    # make heading relative to the ego vehicle
    out_dict["kinematics"]["oxts_heading"] = [
        heading - out_dict["pred_time"]["heading"] for heading in out_dict["kinematics"]["oxts_heading"]
    ]

    # make sure that the keys are valid
    return out_dict


def load_recursively(hdf_obj, keys_to_extract: list = []):
    data = {}
    # Extract attributes
    for name, attr_data in hdf_obj.attrs.items():
        if not keys_to_extract or name in keys_to_extract:
            data[name] = attr_data
    for key in hdf_obj.keys():
        # print(f"Key: {key}")  # Print the key
        if isinstance(hdf_obj[key], h5py.Group):
            # Extract everything in the group
            data[key] = load_recursively(hdf_obj[key], keys_to_extract)
            if not data[key]:
                data.pop(key)
        if isinstance(hdf_obj[key], h5py.Dataset):
            if not keys_to_extract or key in keys_to_extract:
                save_data = hdf_obj[key][()]
                if isinstance(save_data, np.ndarray):
                    data[key] = save_data.tolist()
                else:
                    data[key] = save_data

    return data


def writer(filename, data):
    with open(filename + ".pkl", "wb") as handle:
        pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)


def worker(worker_data):

    file_name, worker_id = worker_data
    # print(f"worker {worker_id} started processing {file_name}")

    wrapper = OSMWrapper()

    df = load_dataset.DatasetFile(file_name)
    group_information = df.get_file_information()

    count = 0
    already_exists = 0
    no_lcm_data = 0
    incomplete_lcm_data = 0
    not_valid = 0
    short_gt = 0
    no_routes_close_by = 0
    other_route_errors = 0
    for data_point in group_information["groups"]:

        # 1. Load sample with data
        data = df.load_sample(data_point)

        data = dict(ele for sub in data.values() for ele in sub.items())
        if data["sequence_id"] + ".pkl" in existing_files:
            print(f"worker {worker_id} skipped {data['sequence_id']}, already exists")
            already_exists += 1
            continue
        if "lcm_data" not in data:
            no_lcm_data += 1
            continue
        if "lcm_lat_acceleration" not in data["lcm_data"]:
            print(f"worker {worker_id} skipped {data['sequence_id']}, lcm data contains only oxts data")
            incomplete_lcm_data += 1
            continue

        # 2. Create a data_out dictionary and add kinematics and ground truth to it
        try:
            data_out = retrieve_kinemetic_data_and_gt(data)
        except ValueError as e:
            if "not valid" in str(e):
                not_valid += 1
                continue

        try:
            data_out = create_route(data_out, wrapper)
        except Exception as e:
            if "Ground truth is less than 200 meters" in str(e):
                short_gt += 1
                continue
            elif "No routes found" in str(e):
                no_routes_close_by += 1
                continue
            else:
                other_route_errors += 1
                print("UNEXPECTED ERROR", e, data["sequence_id"], "SKIPPING!!!", sep="\n")
                if debug:
                    raise
                continue
        count += 1
        writer(f'{output + "/" + data_out["sequence_id"]}', data_out)
        break  # remove later

    del wrapper
    return (
        count,
        no_lcm_data,
        incomplete_lcm_data,
        not_valid,
        already_exists,
        short_gt,
        no_routes_close_by,
        other_route_errors,
    )


def main():

    # Open data folder
    os.makedirs(output, exist_ok=True)

    # main loop
    file_list = os.listdir(folder_name)
    if debug:
        start = 0
        file_list = file_list[start:]

    file_names = [folder_name + "/" + file for file in file_list]
    worker_ids = range(len(file_names))

    # first iterate over filenames and make sure they are not corrupted
    check_corrupted = False
    if check_corrupted:

        def is_corrupted(file_name: h5py.File):
            try:
                df = load_dataset.DatasetFile(file_name)
                df.get_file_information()
                return False
            except:
                return True

        corrupted_files = []
        for out in map(is_corrupted, file_names):
            if out:
                corrupted_files.append(file_names[out])

        print(f"Corrupted files: {corrupted_files}")

        file_names = [file for file in file_names if file not in corrupted_files]

    out_together = {
        "count": 0,
        "no_lcm_data": 0,
        "incomplete_lcm_data": 0,
        "not_valid": 0,
        "already_exists": 0,
        "short_gt": 0,
        "no_routes_close_by": 0,
        "other_route_errors": 0,
    }

    def print_out(out, max_key_length):
        for key, value in out.items():
            print(f"\n{key:<{max_key_length}}: {value}")
        print("\n")
        sys.stdout.flush()

    if debug:
        for f, i in zip(file_names, worker_ids):
            print(f"worker {i} started processing {f}")
            worker((f, i))
        exit()

    max_key_length = max(len(key) for key in out_together.keys())
    try:
        with Pool(workers) as p:
            for out in tqdm(p.imap_unordered(worker, zip(file_names, worker_ids)), total=len(file_names)):
                for i, key in enumerate(out_together.keys()):
                    out_together[key] += out[i]
                print_out(out_together, max_key_length)
    except Exception as e:
        print(f"Error processing files: {e}")
        traceback.print_exc()
    finally:
        print("Final Summary:")
        print_out(out_together, max_key_length)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Pre-process dataset.")

    parser.add_argument("--input", type=str, required=True, help="The input hdf5 folder")
    parser.add_argument("--output", type=str, required=True, help="The output folder")
    parser.add_argument("--workers", type=int, required=False, help="Number of workers")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing files. Remove all existing first.")
    parser.add_argument(
        "--debug", action="store_true", help="Whether to run without multiprocessing. If true, num_workers is ignored."
    )
    parser.add_argument(
        "--check_corrupted",
        action="store_true",
        help="Check if any of already existing files are corrupted. Delete them.",
    )

    args = parser.parse_args()

    # remove files from output
    os.makedirs(args.output, exist_ok=True)
    if args.overwrite:
        for file in os.listdir(args.output):
            os.remove(os.path.join(args.output, file))

    folder_name = args.input
    output = args.output
    existing_files = os.listdir(output)
    debug = args.debug

    if args.workers > 0:
        workers = args.workers
    else:
        workers = 1

    print(f"pickling protocol: {pickle.HIGHEST_PROTOCOL}")
    print(f"Input hdf5 file: {folder_name}")
    print(f"Output pickle folder: {output}")

    load_dataset.load_recursively = load_recursively

    main()
