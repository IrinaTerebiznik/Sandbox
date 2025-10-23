"""Suppose you are given the following code:
class FooBar {
  public void foo() {
    for (int i = 0; i < n; i++) {
      print("foo");
    }
  }
  public void bar() {
    for (int i = 0; i < n; i++) {
      print("bar");
    }
  }
}
The same instance of FooBar will be passed to two different threads:
thread A will call foo(), while thread B will call bar().
 ## Modify the given program to output "foobar" n times ##
Example 1:
        Input: n = 1
        Output: "foobar"
    Explanation: There are two threads being fired asynchronously. One of them calls foo(), while the other calls bar().
    "foobar" is being output 1 time.
Example 2:
        Input: n = 2
        Output: "foobarfoobar"
    Explanation: "foobar" is being output 2 times.
"""

import threading
import argparse
import random
import time


# ---------- Mi solución ----------
class FooBar(object):
    def __init__(self, n):
        self.n = n
        self.foo_event = threading.Event()
        self.bar_event = threading.Event()
        self.foo_event.set()  # arranca habilitado foo
        self.bar_event.clear()  # bar espera

    def foo(self, printFoo):
        for _ in range(self.n):  # range en Py3
            self.foo_event.wait()  # esperar turno
            printFoo()
            self.foo_event.clear()  # cedo mi turno
            self.bar_event.set()  # habilito bar

    def bar(self, printBar):
        for _ in range(self.n):
            self.bar_event.wait()
            printBar()
            self.bar_event.clear()
            self.foo_event.set()


# ---------------------------------


# Callbacks estilo LeetCode   -no le des bola
def printFoo():
    print("foo", end="")


def printBar():
    print("bar", end="")


def run_once(n: int, seed=None, jitter=True):
    if seed is not None:
        random.seed(seed)

    fb = FooBar(n)

    t_foo = threading.Thread(target=fb.foo, args=(printFoo,), name="T-foo")
    t_bar = threading.Thread(target=fb.bar, args=(printBar,), name="T-bar")

    threads = [t_foo, t_bar]
    random.shuffle(threads)  # orden aleatorio de arranque

    for t in threads:
        t.start()
        if jitter:
            time.sleep(random.uniform(0, 0.01))

    for t in threads:
        t.join()


def main():
    parser = argparse.ArgumentParser(description="FooBar harness (LeetCode 1115)")
    parser.add_argument(
        "--n", type=int, default=1, help="Cantidad de 'foobar' a imprimir"
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Semilla aleatoria opcional"
    )
    parser.add_argument(
        "--no-jitter", action="store_true", help="Desactiva jitter de arranque"
    )
    args = parser.parse_args()

    run_once(args.n, seed=args.seed, jitter=not args.no_jitter)
    print()  # newline final


if __name__ == "__main__":
    main()
