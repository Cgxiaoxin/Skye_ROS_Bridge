from pyvitaisdk import GF225, VTSDeviceFinder

finder = VTSDeviceFinder()
if len(finder.get_sns()) == 0:
    print("No device found.")
sn = finder.get_sns()
print(sn)
# arm b left 'GF2251386F6E6'
# arm b right 'GF225135972E9'