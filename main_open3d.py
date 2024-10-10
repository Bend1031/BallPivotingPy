import time

import open3d as o3d

from src.FileIO import FileIO
from src.KDtree import KDtree
from src.Mesher import Mesher
from src.Octree import Octree
from src.OctreeIterator import OctreeIterator


def calculate_time(start_time):
    end_time = time.time()
    return end_time - start_time


def main():
    in_file = r"D:/Code/PointCloud/data/other_test/circle.txt"
    out_file = r"D:/Code/PointCloud/data/other_test/circle_多球_补孔.ply"

    radii = [1, 1.1]
    radii.sort()
    main_radius = radii[0]
    print(f"Main radius: {main_radius}")

    start_time = time.time()
    # Create octree
    octree = Octree()
    # if main_radius > 0:
    #     FileIO.read_and_sort_points(in_file, octree, main_radius)

    # print(f"Octree with depth {octree.getDepth()} created.")
    # print(
    #     f"Octree contains {octree.getNpoints()} points. The bounding box size is {octree.getSize()}"
    # )
    # print(
    #     f"Reading and sorting points in this octree took {calculate_time(start_time):.3f} s."
    # )

    # # Octree statistics
    # octree.printOctreeStat()

    # print("Reconstructing with radii")
    iterator = OctreeIterator(octree)
    # iterator.setR(
    #     main_radius
    # )  # Set the radius for the iterator (used in reconstruction)

    # kdtree
    kdtree = KDtree(in_file)
    # Reconstruct
    mesher = Mesher(octree, iterator, kdtree)
    reconstruct_start_time = time.time()
    mesher.reconstruct(radii)
    print(f"Reconstructing took {calculate_time(reconstruct_start_time):.3f} s.")
    print(
        f"Reconstructed mesh: {mesher.nVertices()} vertices, {mesher.nFacets()} facets."
    )
    print(f"{mesher.nBorderEdges()} border edges")

    fill_holes_start_time = time.time()
    mesher.fillHoles()
    print(f"Filled holes in {calculate_time(fill_holes_start_time):.3f} s.")
    print(
        f"Final mesh: {mesher.nVertices()} vertices, {mesher.nFacets()} facets,{mesher.nBorderEdges()} border edges."
    )
    FileIO.save_mesh(out_file, mesher)


if __name__ == "__main__":
    # cProfile.run("main()", "profile.txt", sort="cumulative")
    main()
