class Customer:
    """
    Class to represent a customer instance.
    """
    def __init__(self, id, location, wt, drone_eligible=True):
        """Initialize a customer instance
        args: type, description:
        - id: int, identifyer of the customer
        - location: list of 2 floats, lat lon location of the customer
        - wt: float, weight of the customer's package in lbs
        """
        self.id = id
        self.location = location
        self.wt = wt
        self.drone_eligible = drone_eligible
        self.delivery_time = None

    def __repr__(self):
        """Get the data within the customer instance"""
        return (f"Customer(id={self.id}, location={self.location}, "
                f"drone_eligible={self.drone_eligible}, delivery_time={self.delivery_time}), "
                f"weight={self.wt}")
    
    def to_dict(self):
        """Convert the customer data to a dictionary"""
        return {
            'id': self.id,
            'location': self.location.tolist(),
            'served': self.served,
            'weight': self.wt,
            'drone_eligible': self.drone_eligible,
            'delivery_time': self.delivery_time
        }

class Cluster():
    """
    Class to represent a cluster instance.
    """
    def __init__(self, id, centroid, customers, served=False):
        self.id = id
        self.centroid = centroid
        self.customers = customers
        self.served = served

    def mark_served(self):
        """Mark a customer as served
        args: type, description:
        - delivery_time: float, time of delivery"""
        self.served = True