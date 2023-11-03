from pyproj import Proj

p = Proj(proj='utm',zone=50,ellps='WGS84', preserve_units=False)

x = (389087.76814579882,389087.99845364108,389090.60391104134,389090.37350324087)
y = (3155572.7071234803,3155566.8880520388,3155566.99121076,3155572.8102821992)
lons, lats = p(x, y,inverse=True)

print('lons: %9.14f %9.14f %9.14f %9.14f' % lons)
print('lats: %9.14f %9.14f %9.14f %9.14f' % lats)

print('lons: %9.14f,%9.14f %9.14f,%9.14f %9.14f,%9.14f %9.14f,%9.14f' % (lons[0],lats[0],lons[1],lats[1],lons[2],lats[2],lons[3],lats[3]))
