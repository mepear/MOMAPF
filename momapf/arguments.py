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
        "--use-caching", default='False', type=str, help="whether use caching or not"
    )
    parser.add_argument(
        "--index", default="0",  type=str, help="index for choosing start and goal locations"
    )
    parser.add_argument(
        "--heuristic-name", type=str, default='perfect', help="heuristic name for experiment"
    )
    parser.add_argument(
        "--robot-num", default=0, type=int, help="number of moveable robots"
    )
    parser.add_argument(
        "--cost-name", type=str, nargs='+', help="cost name list"
    )
    parser.add_argument(
        "--draw-graph", default='False', type=str, help="whether draw graph"
    )
    args = parser.parse_args(args) if args is not None else parser.parse_args()

    return args
