def greet(name: str) -> str:
    return f"hello, {name}"


def join_args(*items: str) -> str:
    return ",".join(items)
