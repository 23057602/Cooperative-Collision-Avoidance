# spline_works

A library package for defining and evaluating cubic splines. Cubic splines are smooth parametric curves. This implementation approaches them as polynomials encoded by matrices with a domain of $t \in \left[0,1\right]$ for the input parameter $t$. Both Python and C++ are supported.

## Installation & Dependencies
This package depends on:
- C++:
    - Eigen3
    - GSL
- Python:
    - NumPy
    - SciPy

To install, please copy the directory into the workspace and install with a build tool. This package was produced with a ROS2 toolchain as a C++/CMake package for colcon using `ament_cmake` and `ament_python`:
```bash
colcon build --package-select spline_works
```

## Usage
Inclusion in C++:
```C++
#include <spline_works/spline_works.hpp>
```
Import in Python:
```Python
import spline_works
```

## Classes and key members
- **Spline**
    - C : Characteristic matrix.
    - M : Control matrix.
    - len : Length of the curve.
    - (*float*) : Returns the point along curve at the passed parameter value.
    - getClosestT(*Vector*) : Returns the parameter value of the point along the curve closest to the point passed in.
    - getTangent(*float*) : Returns the vector tangent to the curve at the point the passed parameter value specifies.
    - getLength(*float*) : Returns the length along the curve between the start of the curve and the point the passed parameter value specifies.
- **Splines**
    - (*float*) : Returns point the along curve at the passed parameter value. The integer part of *float* indexes the specific curve.
    - getClosestT(*Vector*) : Returns the parameter value of the point along the curve closest to the point passed in. The integer part indexes the specific curve.
    - getTangent(*float*) : Returns the vector tangent to the curve at the point the passed parameter value specifies. The integer part of *float* indexes the specific curve.
- **Spline_make**
    - hermite_mtx : A characteristic matrix of the Hermite spline
    - herm_line(...) & herm_arc2d(...) : Returns control point matrix for a line/arc base on given parameters.
    - fromPoints(*Matrix*,*Matrix*) : Returns the control matrix of a curve fitted to the points passed in.
    - search_tol_arg(*Matrix*,*Matrix*,*float*) : Returns the amount of points that can be fitted to a curve within a tolerance of *float*.
    - herm_seg(*Matrix*,*float*,*float*) : Returns the control point matrix corresponding to the segment between the two *float* parameter values passed.
    - serialize(...)/deserialize(...) : Flattens/Reshapes the control matrix passed.