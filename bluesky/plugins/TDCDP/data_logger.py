import logging
import json
import os
import numpy as np
class DataLogger:
    def __init__(self, log_filename='simulation_log.jsonl', log_dir='bluesky/plugins/TDCDP/datalogs'):
        self.log_filename = log_filename
        self.log_dir = log_dir
        self.logger = logging.getLogger('DataLogger')
        self.logger.setLevel(logging.INFO)
        self._configure_logger()

    def _configure_logger(self):
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
        json_data = json.dumps(data)
        self.logger.info(json_data)

    def log_customer_service(self, data, acid, idx, cust_count):
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
            "coordinates": coords
        }

        self.log_data(log_entry)

    def shutdown(self):
        logging.shutdown()

    def check_log_file(self):
        log_path = os.path.join(self.log_dir, self.log_filename)
        if os.path.exists(log_path):
            print(f"Log file '{log_path}' created successfully.")
        else:
            print(f"Failed to create log file '{log_path}'.")

    def change_log_file(self, new_filename=None, new_log_dir=None):
        # Update filename and/or path if provided
        if new_filename:
            try:
                ext = new_filename.split('.')[1]
                if ext != 'jsonl':
                    new_filename = new_filename.split('.')[0] + '.jsonl'
            except IndexError:
                new_filename = new_filename + '.jsonl' 
            self.log_filename = new_filename
        if new_log_dir:
            self.log_dir = new_log_dir
        
        # Reconfigure logger with new settings
        self._configure_logger()