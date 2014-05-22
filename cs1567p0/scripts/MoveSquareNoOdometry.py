from CommandUtils import move_forward, stop_command, rotate_command
if __name__ == "__main__":
    rotate_amount = 90
    if len(sys.argv) > 1:
        rotate_amount = float(sys.argv[1])

    init_commands('MoveSquareNoOdometry')
    for i in range(4):
        move_forward(1.0, 7.0)
        stop_command()
        rotate_command(rotate_amount)
        stop_command()
