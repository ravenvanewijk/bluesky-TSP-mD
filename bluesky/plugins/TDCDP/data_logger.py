import logging
import json
import os
import re
import numpy as np

class DataLogger:
    def __init__(self, log_filename='simulation_log.jsonl', log_dir='bluesky/plugins/TDCDP/datalogs/default'):
        """Initialize the DataLogger class.
        
        args: type, description:
            - log_filename: str, the name of the log file (default is 'simulation_log.jsonl')
            - log_dir: str, the directory where log files will be stored (default is 'bluesky/plugins/TDCDP/datalogs/default')
        """
        self.log_filename = log_filename
        self.log_dir = log_dir
        self.logger = logging.getLogger('DataLogger')
        self.logger.setLevel(logging.INFO)
        self._configure_logger()

    def _configure_logger(self):
        """Configure the logger settings, create log directory if it does not exist, and set up file handler."""
        # Create log directory if it does not exist
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # Set the full log path
        log_path = os.path.join(self.log_dir, self.log_filename)
        
        # Remove existing handlers
        while self.logger.hasHandlers():
            self.logger.removeHandler(self.logger.handlers[0])

        # Create a file handler
        file_handler = logging.FileHandler(log_path)
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(logging.Formatter('%(message)s'))

        # Add the file handler to the logger
        self.logger.addHandler(file_handler)

    def log_data(self, data):
        """Log data to the configured log file.
    
        args: type, description:
            - data: dict, the data to be logged in dictionary format
        """
        json_data = json.dumps(data)
        self.logger.info(json_data)

    def log_customer_service(self, data, acid, idx, cust_count, op_time):
        """Log a customer service entry to the log file.

        args: type, description:
            - data: dict, the operational state data for the vehicle
            - acid: str, the identifier for the vehicle (e.g., 'TRUCK')
            - idx: int, the index of the operation to be logged
            - cust_count: int, the customer ID being served
        """
        t_0 = data['t0'][idx]
        op_duration = data['op_duration'][idx]
        coords = data['wp_coords']
        if t_0 == np.inf:
            t = "Infinity"  # Handle inf case for logging purposes
        else:
            t = t_0 + op_duration
        
        log_entry = {
            "customer_id": cust_count,
            "served_time": t,
            "vehicle": acid,
            "coordinates": coords,
            "op_time": op_time
        }

        self.log_data(log_entry)

    def log_droneop(self, op_type, waiting_entity, waiting_time, op_time):
        log_entry = {
            "op_type": op_type,
            "waiting_entity": waiting_entity,
            "waiting_time": waiting_time,
            "op_completion_time": op_time
        }

        self.log_data(log_entry)

    def shutdown(self):
        """Shut down the logging system to ensure all log entries are properly written and closed.∂"""
        logging.shutdown()

    def check_log_file(self):
        """Check if the log file exists and print the result."""
        log_path = os.path.join(self.log_dir, self.log_filename)
        if os.path.exists(log_path):
            print(f"Log file '{log_path}' created successfully.")
        else:
            print(f"Failed to create log file '{log_path}'.")

    def change_log_file(self, new_filename=None, new_log_dir=None):
        """Change the log file and/or log directory, and reconfigure the logger.
        
        args: type, description:
            - new_filename: str, optional, the new name of the log file
            - new_log_dir: str, optional, the new directory where the log file will be stored
        """
        # Update filename and/or path if provided
        if new_filename:
            # Define a regular expression pattern to match the 
            # filename and capture the drone config, solve method, and uncertainty
            pattern = r'^tbl_solutions_(\d+)_(\d+)_(\w+)_(\w+)$'

            # Use re.match to check if the filename matches the pattern
            match = re.match(pattern, new_filename)
            if match:
                new_log_dir = new_log_dir + '/' + match.group(1) + '/' + match.group(2)
            try:
                ext = new_filename.split('.')[1]
                if ext != 'jsonl':
                    new_filename = new_filename.split('.')[0] + '.jsonl'
            except IndexError:
                new_filename = new_filename + '.jsonl' 
            self.log_filename = new_filename
        if new_log_dir:
            self.log_dir = 'bluesky/plugins/TDCDP/datalogs/' + new_log_dir
        
        # Reconfigure logger with new settings
        self._configure_logger()