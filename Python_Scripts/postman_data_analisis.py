# Create a venv and install panda using pip install pandas openpyxl      
# B4 executing


import json
import os
import pandas as pd
from datetime import datetime


def process_json(file_path):
    """
    Processes a single JSON file, extracts relevant mission details from the
    'body' field, and returns a list of dictionaries.
    """
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    results = []
    for item in data:
        if isinstance(item, dict):  # Ensure 'item' is a dictionary
            try:

                body_data = json.loads(item.get("body", "{}"))


                mission_detail = body_data.get("data", {})
                time_24hs = mission_detail.get("vehicleTime").split(", ")[1]
                # Create the result dictionary with desired data
                mission_data = {
                    "Time": time_24hs,
                    "ID": mission_detail.get("missionTaskDetail").get("ID"),
                    "mode": mission_detail.get("missionTaskDetail").get("mode"),
                    "status": mission_detail.get("missionTaskDetail").get("status"),
                }
                results.append(mission_data)
            except json.JSONDecodeError:
                print(
                    f"Warning: 'body' field in {item.get('name', 'unknown')} "
                    f"from {file_path} is not a valid JSON string. Skipping."
                )
            except Exception as e:
                print(f"Error processing item in {file_path}: {e}")
        else:
            print(
                f"Warning: Expected a dictionary in {file_path}, but got a "
                f"{type(item)}. Skipping."
            )
    return results


def process_all_json_files(directory, output_excel_name="mission_data.xlsx"):
    """
    Processes all JSON files in a given directory, aggregates the data,
    and saves it to a single Excel file.
    """
    all_mission_data = []
    processed_count = 0

    if not os.path.exists(directory):
        print(f"Error: Directory '{directory}' not found.")
        return

    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            print(f"Processing file: {filename}")

            # Extend the main list with results from each file
            all_mission_data.extend(process_json(file_path))
            processed_count += 1

    if not all_mission_data:
        print(f"No valid JSON data found in '{directory}' to process.")
        return

    # Create a pandas DataFrame from the collected data
    df = pd.DataFrame(all_mission_data)

    # Define the output Excel file path
    output_excel_path = os.path.join(directory, output_excel_name)

    try:
        # Save the DataFrame to an Excel file
        df.to_excel(output_excel_path, index=False, engine="openpyxl")
        print(f"\nSuccessfully saved all mission data to: {output_excel_path}")
        print(f"Processed {processed_count} JSON files.")
    except Exception as e:
        print(f"Error saving data to Excel file {output_excel_path}: {e}")


# Path where your JSON files are located
directory_to_process = "logs"  # Change this path as needed

# Define the name of the output Excel file
excel_output_filename = (
    f"mission_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
)

# Call the function to process all files and generate the Excel report
process_all_json_files(directory_to_process, excel_output_filename)
