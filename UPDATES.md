# Robot collision teams status updates

## April 12th, 2023

- Riley: Rai is now working. We are able to compile and run our code through rai with a basic collision test. I will begin working on the broad phase kernel.
- Nandeeka: Python version of the narrow phase implemented. It's extremely compute intensive. Will start the rai implementation next.
- Victor: The transformation kernel (transform the robot given a configuration) is now compiling and running with rai, but its correctness is currently unknown. Will run some tests next.
- Aakash: Nested for loop version for AABB kernel is generated (axis-aligned bounding boxes). Will push later tonight to the github and then start work on the GPU version.

# April 19th, 2023

- Riley: Broad phase kernel now has a basic implementation (no shared memory is used yet). I will test it once the AABB kernel is working, then optimize.
- Nandeeka: I am currently working on the C++ implementation of the narrow phase. While LOC is often an unsatisfying metric, I will say that I spent many hours and wrote >750 LOC. Currently, very close to having a working implementation (minus all of the checks necessary if the triangles are coplanar). I think the easiest will be for us to proceed with this implementation now, raising an error if coplanar triangles are encountered.
- Victor: Transformation kernel implemented, tested for correctness, and benchmarked. For each vertex, I was able to get the absolute value of their differences on the order of 1e-5, which I suspect is good enough for correctness, though I would appreciate some feedback on this. As for performance, this part is phenomenal so far. The GPU version was able to load, run, and transfer 10,000 configurations at about 170x the rate of the CPU. There are some optimizations I could make -- for the CPU we could compute the transformation matrix directly instead of multiplying the translation matrix by the three rotation matrices, and for the GPU we could use shared memory tiling to coalesce global memory writes -- but overall, this shows a great deal of potential for the algorithm overall.
- Aakash: I am working on the kernel to generate the AABBs. I have a very basic, unoptimized parallel kernel ready and this kernel can be used temporarily while I finish the optimized version. This kernel and the serial logic have been tested and verified. I am currently working on the optimized parallel version and it is almost ready. The optimized kernel uses a parallel reduction technique to find the AABB for each configuration.

# April 26, 2023

- Riley: Worked on the broad phase kernel, basic test (robot collides with itself) working; priority for next week: full end-to-end integration test
- Victor: Trying to merge everything into one branch, just worked on getting build file working
- Aakash: By the meeting, he will be able to integrate his test into Victor's
- Nandeeka: Finished the CPU implementation and the basic GPU implementation. Goals for next week: test on real configurations (for profiling); optimize by moving accesses to shared memory or even registers
