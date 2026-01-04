import pytest

def test_mission_scoring():
    # Senior scoring simulation
    # Mission 1
    seeds_correct = 8  # +5 each
    seeds_fully_within = 6  # +10 each (subset of correct)
    seeds_misplaced = 2  # -5 each
    plots_irrigated_correct = 2  # +30 each
    plots_irrigated_wrong = 1  # -10 each

    # Mission 2
    fruits_moved_red_black = 8  # +5 each
    red_in_fruits = 5  # +5 each
    red_in_waste = 1  # -5 each
    black_in_waste = 3  # +10 each
    black_in_fruits = 0  # -10 each
    green_moved = 1  # -5 each

    score_m1 = (seeds_correct * 5) + (seeds_fully_within * 10) + (seeds_misplaced * (-5)) + (plots_irrigated_correct * 30) + (plots_irrigated_wrong * (-10))
    score_m2 = (fruits_moved_red_black * 5) + (red_in_fruits * 5) + (red_in_waste * (-5)) + (black_in_waste * 10) + (black_in_fruits * (-10)) + (green_moved * (-5))

    total_score = score_m1 + score_m2

    assert total_score > 0
    print(f'Mission 1 score: {score_m1}, Mission 2 score: {score_m2}, Total score: {total_score}')

if __name__ == '__main__':
    test_mission_scoring()