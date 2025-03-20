import os
import glob

def delete_data(directory):
    # Find all CSV files in the directory
    csv_files = glob.glob(os.path.join(directory, "*.csv"))

    # Loop through the list of CSV files and delete each file
    num_deleted = 0
    for file_path in csv_files:
        try:
            os.remove(file_path)
            num_deleted += 1
        except Exception as e:
            print(f"Error deleting {file_path}: {e}")
    print(f"Deleted {num_deleted} files")

if __name__ == "__main__":
    delete_data("data/state")
    delete_data("data/action")