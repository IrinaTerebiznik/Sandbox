"""
Suppose we have a class:
    public class Foo {
    public void first() { print("first"); }
    public void second() { print("second"); }
    public void third() { print("third"); }
    }
The same instance of Foo will be passed to three different threads. Thread A will call first(), thread B will call second(), and thread C will call third(). Design a mechanism and modify the program to ensure that second() is executed after first(), and third() is executed after second().
Note:
We do not know how the threads will be scheduled in the operating system, even though the numbers in the input seem to imply the ordering. The input format you see is mainly to ensure our tests' comprehensiveness.
Example 1:
    Input: nums = [1,2,3]
    Output: "firstsecondthird"
Explanation: There are three threads being fired asynchronously. The input [1,2,3] means thread A calls first(), thread B calls second(), and thread C calls third(). "firstsecondthird" is the correct output.
Example 2:
    Input: nums = [1,3,2]
    Output: "firstsecondthird"
Explanation: The input [1,3,2] means thread A calls first(), thread B calls third(), and thread C calls second(). "firstsecondthird" is the correct output.
"""

import threading
import argparse
import random
import time

# ---------- Mi solución ----------


class Foo(object):
    def __init__(self):
        self.s1 = threading.Semaphore(0)
        self.s2 = threading.Semaphore(0)

    def first(self, printFirst):
        printFirst()
        self.s1.release()

    def second(self, printSecond):
        self.s1.acquire()
        printSecond()
        self.s2.release()

    def third(self, printThird):
        self.s2.acquire()
        printThird()


# ---------------------------------


# Esto es lo que hacia LeetCode -- no le des bola
def printFirst():
    print("first", end="")


def printSecond():
    print("second", end="")


def printThird():
    print("third", end="")


def run_once(nums, seed=None):
    """
    nums: lista de 3 enteros con valores {1,2,3} en cualquier orden.
          1 -> thread llama first(), 2 -> second(), 3 -> third()
    """
    if seed is not None:
        random.seed(seed)

    f = Foo()

    # Mapa: número -> (target, args)
    targets = {
        1: (f.first, (printFirst,)),
        2: (f.second, (printSecond,)),
        3: (f.third, (printThird,)),
    }

    threads = []
    for n in nums:
        t = threading.Thread(target=targets[n][0], args=targets[n][1], name=f"T{n}")
        threads.append(t)

    # Opcional: mezclar latencias para simular scheduling impredecible
    random.shuffle(threads)

    for t in threads:
        t.start()
        # Pequeño jitter opcional para enfatizar la concurrencia
        time.sleep(random.uniform(0, 0.01))

    for t in threads:
        t.join()


def main():
    parser = argparse.ArgumentParser(description="Foo ordering harness (LeetCode 1114)")
    parser.add_argument(
        "--nums",
        nargs="+",
        type=int,
        default=[1, 2, 3],
        help="Orden de llegada de los threads, e.g. --nums 1 3 2",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Semilla aleatoria opcional"
    )
    args = parser.parse_args()

    run_once(args.nums, seed=args.seed)
    print()  # newline final


if __name__ == "__main__":
    main()
