import argparse

def get_args(args):
    parser = argparse.ArgumentParser(description='MOMAPF')
    parser.add_argument(
        '--experiment-name', default='default', help='experiment map name'
    )
    parser.add_argument(
        "--use-cost-bound", default='False', type=str, help="whether use cost bound or not"
    )
    parser.add_argument(
        "--use-joint-splitting", default='False', type=str, help="whether use joint splitting or not"
    )
    parser.add_argument(
        "--index", default="0",  type=str, help="index for choosing start and goal locations"
    )
    parser.add_argument(
        "--heuristic-name", default=False, help="heuristic name for experiment"
    )
    parser.add_argument(
        "--robot-num", default=0, type=int, help="number of moveable robots"
    )
    parser.add_argument(
        "--cost-name", type=str, nargs='+', help="cost name list"
    )
    args = parser.parse_args(args) if args is not None else parser.parse_args()

    return args
