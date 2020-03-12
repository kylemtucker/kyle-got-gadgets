import suncalcPy.suncalc as sc
from datetime import datetime
import math
















if __name__ == "__main__":
    pos = sc.getPosition(datetime.now(), 37.86948, -122.25929)
    print("altitude: ")
    print(pos["altitude"] * 180)
    print("\n azimuth")
    print(pos["azimuth"] * 180)
    
    print(pos)
