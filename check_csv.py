import pandas as pd
import os
import glob

def find_nonzero_files(directory, column_name):
    """
    Returns a list of CSV filenames where the specified column 
    contains no zeros across all rows.
    """
    # Create a path pattern for all .csv files
    path_pattern = os.path.join(directory, "*.csv")
    csv_files = glob.glob(path_pattern)
    valid_files = []

    for file_path in csv_files:
        try:
            # usecols optimizes performance by only loading the relevant column
            df = pd.read_csv(file_path, usecols=[column_name])
            
            # Check if all values in the column are NOT equal to 0
            # Note: .all() returns True only if the condition holds for every row
            if  (df[column_name] == 1).any():
                valid_files.append(os.path.basename(file_path))
                
        except ValueError:
            # This handles cases where the column name doesn't exist in the file
            print(f"Skipping {file_path}: Column '{column_name}' not found.")
        except Exception as e:
            print(f"Could not process {file_path}: {e}")

    return valid_files

# --- CONFIGURATION ---
folder_to_scan = './logs/ecvt'           # Path to your CSV folder
target_column = 'centerlock_inbound_limit_switch'   # The column name you want to check

# Execution
matching_files = find_nonzero_files(folder_to_scan, target_column)

print("\n--- Files where the column is never 0 ---")
if matching_files:
    for filename in matching_files:
        print(filename)
else:
    print("No matching files found.")