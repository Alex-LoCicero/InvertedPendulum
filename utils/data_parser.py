class DataParser:
    def __init__(self):
        self.data = {}

    def parse(self, raw_data) -> dict:
        # Example format: "DATA, timestamp:1234567890, motor_sp:100, motor_pos:50, joint_pos:30"
        if "DATA" not in raw_data:
            return None
        data_points = raw_data.split(', ')
        for point in data_points:
            if ':' in point:
                key, value = point.split(':')
                self.data[key.strip()] = float(value.strip())
        return self.data