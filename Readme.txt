Implicit Hoppe reconstruction
I iterated through all the points to find the closest point, then compute the distance to the plane defined by the plane and normal.

ImplicitRBF reconstruction
I find the bounding box and used diagnal * 0.01 as epsilon
Then I constructed the linear system that defines the signed distance field to the surface based on RBF. Solved the system to find the best weight