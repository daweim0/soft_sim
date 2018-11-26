# from data_dump import data as points
import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import ast
import numpy as np
from numpy.linalg import svd


# from https://stackoverflow.com/questions/12299540/plane-fitting-to-4-or-more-xyz-points
def planeFit(points):
    """
    p, n = planeFit(points)

    Given an array, points, of shape (d,...)
    representing points in d-dimensional space,
    fit an d-dimensional plane to the points.
    Return a point, p, on the plane (the point-cloud centroid),
    and the normal, n.
    """
    points = np.reshape(points, (np.shape(points)[0], -1))  # Collapse trialing dimensions
    ctr = points.mean(axis=1)
    try:
        assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
        x = points - ctr[:,np.newaxis]
        M = np.dot(x, x.T) # Could also use np.cov(x) here.
        normal = svd(M)[0][:, -1]
        normal /= np.linalg.norm(normal)
        if normal[0] < 0:
            normal = normal * -1  # make all normal vectors point in the positive x direction
        return ctr, normal
    except:
        return ctr, np.array([1, 0, 0])


def swap_yz(arr):
    return np.array([arr[0], arr[2], arr[1], arr[3], arr[5], arr[4]])


num_output_points = 1
num_input_points = 3

def main():

    file = open("../triangle_output.txt", "r")
    input_points = []
    output_points = []
    for line in file:
        all_points = ast.literal_eval("[" + line.replace("\n", "")[0:-1] + ']')[1:]
        input_points.append(all_points[:num_input_points])
        output_points.append(all_points[-num_output_points:])


    data_filtered = []
    # calculate normal to output plane
    for i in range(len(output_points)):
        center, normal = planeFit(np.array(output_points[i])[:, 0:3].transpose())  # only pass in the x, y, and z coordinates, not the orientation
        data_filtered.append(np.concatenate((center, normal)))
        pass


    x, y, z, dx, dy, dz = np.array(data_filtered).transpose()

    dx, dy, dz = np.array([dx, dy, dz]) / 500

    data_array = np.array([x, y, z, dx, dy, dz])
    characteristic_set = []

    for i in range(data_array.shape[0]):
        characteristic_set.append(data_array.transpose()[np.argmax(data_array[i])])
        characteristic_set.append(data_array.transpose()[np.argmin(data_array[i])])

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    vectors_mat = np.array(characteristic_set)

    # do_pca(vectors_mat)
    max_volume_submat(vectors_mat)

    characteristic_set_transposed = np.array(vectors_mat).transpose()
    characteristic_set_transposed = swap_yz(characteristic_set_transposed)

    ax.quiver(*characteristic_set_transposed, colors="red", length=0.005)

    # add the fixed points to the display
    base_xyz = swap_yz(np.array(input_points[0]).transpose()).transpose()
    for i in range(base_xyz.shape[0]):
        ax.quiver(base_xyz[i][0], base_xyz[i][1], base_xyz[i][2], base_xyz[i][3], base_xyz[i][4], base_xyz[i][5], colors="blue", length=0.002)

    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')
    plt.show()



def do_pca(data_mat : np.ndarray):
    print("input mat:")
    print(data_mat)
    print()

    U, s, Vt = np.linalg.svd(data_mat, full_matrices=False)
    V = Vt.T
    print("U:")
    print(U)
    print()
    print("s:")
    print(s)


def max_volume_submat(data_mat: np.ndarray, k=3):
    print("input mat:")
    print(data_mat)
    print()

    vector_iterable = []
    for i in range(data_mat.shape[0]):
        vector_iterable.append(data_mat[i])

    S = []

    def vector_norm(vect):
        return vect, np.linalg.norm(vect)

    def subtract_projection(a, b):
        """ subtracts the projection of a from b and returns the new b """
        return b - np.dot(a, b) / np.linalg.norm(b) * (b / np.linalg.norm(b))

    for i in range(k):
        min_vect, mag = max(map(vector_norm, vector_iterable), key=lambda x: x[1])
        vector_iterable_temp = filter(lambda x: (min_vect != x).any(), vector_iterable)
        vector_iterable = list(map(lambda x: subtract_projection(min_vect, x), vector_iterable_temp))
        S.append(min_vect)
        print("iter " + str(i) + " mag:", mag)

    print(S)
    return S


if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    # test code
    # max_volume_submat(np.array([[1, 0, 0, 0, 0],
    #                             [0, 1, 0, 0, 0],
    #                             [0, 0, 1, 0, 0],
    #                             [0, 0, 0, 0.1, 0],
    #                             [0, 0, 0, 0, -0.1]]))

    main()