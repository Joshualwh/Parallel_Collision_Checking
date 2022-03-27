import fcl
import numpy as np

g1 = fcl.Box(1,1,1)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)

T = np.array([1.0, 2.0, 3.0])

g2 = fcl.Cone(1,3)
t2 = fcl.Transform(T)
o2 = fcl.CollisionObject(g2, t2)

request = fcl.CollisionRequest()
result = fcl.CollisionResult()

ret = fcl.collide(o1, o2, request, result)

print (ret)