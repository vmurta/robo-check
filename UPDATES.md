# Robot collision teams status updates

## April 12th, 2023

- Riley: Rai is now working. We are able to compile and run our code through rai with a basic collision test. I will begin working on the broad phase kernel.
- Nandeeka: Python version of the narrow phase implemented. It's extremely compute intensive. Will start the rai implementation next.
- Victor: The transformation kernel (transform the robot given a configuration) is now compiling and running with rai, but its correctness is currently unknown. Will run some tests next.
- Aakash: Nested for loop version for AABB kernel is generated (axis-aligned bounding boxes). Will push later tonight to the github and then start work on the GPU version.

# April 19th, 2023

- Riley: Broad phase kernel now has a basic implementation (no shared memory is used yet). I will test it once the AABB kernel is working, then optimize.
- Nandeeka: I am currently working on the C++ implementation of the narrow phase. While LOC is often an unsatisfying metric, I will say that I spent many hours and wrote >750 LOC. Currently, very close to having a working implementation (minus all of the checks necessary if the triangles are coplanar). I think the easiest will be for us to proceed with this implementation now, raising an error if coplanar triangles are encountered.
- Victor: Transformation kernel implemented, tested for correctness, and benchmarked. For each vertex, I was able to get the absolute value of their differences on the order of 1e-5, which I suspect is good enough for correctness, though I would appreciate some feedback on this. As for performance, this part is phenomenal so far. The GPU version was able to load, run, and transfer 10,000 configurations at about 170x the rate of the CPU. There are some optimizations I could make -- for the CPU we could compute the transformation matrix directly instead of multiplying the translation matrix by the three rotation matrices, and for the GPU we could use shared memory tiling to coalesce global memory writes -- but overall, but overall this shows a great deal of potential for the algorithm overall.
