import numpy as np


def signum(x):
    if x == 0:
        return 0
    elif x < 0:
        return -1
    else:
        return 1


def intbound(s, ds):
    if (ds < 0):
        return intbound(-s, -ds)
    else:
        s = s % 1
        return (1 - s) / ds


class point:
    def __init__(self, pt):
        self.x = pt[0]
        self.y = pt[1]
        # self.z = z

    def to_np(self):
        return np.array([self.x, self.y])


def raycast(start, end, lmin, lmax):
    x = int(np.floor(start.x))
    y = int(np.floor(start.y))
    endX = int(np.floor(end.x))
    endY = int(np.floor(end.y))
    maxDist = np.sum((end.to_np() - start.to_np()) ** 2)

    dx = endX - x
    dy = endY - y

    stepX = signum(dx)
    stepY = signum(dy)

    tMaxX = intbound(start.x, dx)
    tMaxY = intbound(start.y, dy)

    tDeltaX = stepX / dx
    tDeltaY = stepY / dy

    output = []

    if (stepX == 0 and stepY == 0):
        return output

    dist = 0
    while True:
        if lmin.x <= x < lmax.x and lmin.y <= y < lmax.y:
            output.append(np.array([x, y]))

            dist = np.sum( (np.array([x,y]) - start.to_np()) ** 2)

            if (dist > maxDist):
                break
                # return output


            if len(output) > 1500:
                return None

            if x == endX and y == endY:
                break

            if tMaxX < tMaxY:
                x += stepX
                tMaxX += tDeltaX
            else:
                y += stepY
                tMaxY += tDeltaY
    return output


def plot_demo(start, end, lmin, lmax):
    import matplotlib.pyplot as plt
    fig1 = plt.figure()
    xx = [start.x, end.x]
    yy = [start.y, end.y]
    plt.plot(xx, yy)

    for i in range(lmin.x, lmax.x + 1):
        plt.plot([i, i], [lmin.y, lmax.y], 'black')
    for j in range(lmin.y, lmax.y + 1):
        plt.plot([lmin.x, lmax.x], [j, j], 'black')

    # plt.grid()
    # plt.axis('equal')
    plt.xlim([lmin.x, lmax.x])
    plt.ylim([lmin.y, lmax.y])
    plt.show()


if __name__ == "__main__":
    start = point([1.2, 0.4])
    end = point([3.7, 3.9])
    lmin = point([0, 0])
    lmax = point([5, 4])
    output = raycast(start, end, lmin, lmax)
    print(output)
    plot_demo(start, end, lmin, lmax)
