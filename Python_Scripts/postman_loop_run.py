import time
import subprocess
import os
import json
from datetime import datetime

def decode_buffer(data_list):
    """Converts a list of bytes (Buffer) into a UTF-8 string."""
    return bytes(data_list).decode("utf-8")


def run_queries():
    """
    Executes a Postman collection via Newman, extracts response bodies,
    and saves them to a JSON file. The full Newman report is not saved.
    """
    now = datetime.now()
    print(f"[{now}] Starting Newman query...")
    os.makedirs("logs", exist_ok=True)

    # Define temporary and final output file paths
    timestamp = now.strftime('%Y-%m-%d_%H-%M-%S')
    temp_output_json = f"logs/temp_result_{timestamp}.json"
    output_body = f"logs/body_{timestamp}.json"

    try:
        # Construct the Newman command as a list of arguments
        newman_command = [
            "newman",
            "run",
            "Cenobot.postman_collection.json",
            "--environment",
            "Cenobot_env.postman_environment.json",
            "--reporters",
            "json",
            "--reporter-json-export",
            temp_output_json,
        ]

        # Execute the Newman command
        subprocess.run(
            newman_command,
            stdout=subprocess.DEVNULL, # Suppress Newman's console output
            stderr=subprocess.DEVNULL, # Suppress Newman's error output
            check=True # Raise an exception if Newman fails
        )
        print(f"[{now}] Newman command executed successfully.")

        # Give a brief moment for the file to be written
        time.sleep(0.5)

        # Process the temporary Newman output file
        if os.path.exists(temp_output_json):
            try:
                with open(temp_output_json, "r", encoding="utf-8") as f:
                    data = json.load(f)

                bodies = []
                for exec_item in data.get("run", {}).get("executions", []):
                    response_stream = \
                        exec_item.get("response", {}).get("stream", {})
                    if (response_stream.get("type") == "Buffer"
                            and "data" in response_stream):
                        decoded_body = decode_buffer(response_stream["data"])
                        bodies.append(
                            {"name": exec_item["item"]["name"],
                             "body": decoded_body}
                        )

                with open(output_body, "w", encoding="utf-8") as f:
                    json.dump(bodies, f, indent=2)

                print(f"[{now}] Saved response bodies to {output_body}")

            except json.JSONDecodeError:
                print(f"[{now}] Error: Could not decode JSON from "
                      f"{temp_output_json}. File might be corrupt or empty.")
            except Exception as e:
                print(f"[{now}] Error processing response body: {e}")
            finally:
                # Always remove the temporary full Newman output file
                if os.path.exists(temp_output_json):
                    os.remove(temp_output_json)
                    print(f"[{now}] Cleaned up temporary file: "
                          f"{temp_output_json}")
        else:
            print(f"[{now}] Temporary Newman output file not found: "
                  f"{temp_output_json}. Newman might have failed "
                  f"to generate it.")

    except subprocess.CalledProcessError as e:
        print(f"[{now}] Error running Newman command: {e}")
        print(f"[{now}] Check Newman installation and collection/environment "
              f"paths.")
    except FileNotFoundError:
        print(f"[{now}] Error: 'newman' command not found. Make sure Newman "
              f"is installed and in your system's PATH.")
    except Exception as e:
        print(f"[{now}] An unexpected error occurred during Newman execution:"
              f" {e}")


def main():
    """
    Main function to run the query process continuously every 30 seconds.
    """
    print("⏳ Starting continuous query loop...")
    while True:
        run_queries()
        print("-" * 30) # Separator for clarity in console output
        time.sleep(30) # Wait for 30 seconds before the next run


if __name__ == "__main__":
    main()