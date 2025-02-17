"""This script adds line comments to an include-what-you-use output file to enable automatic removal of code lines
by fix_includes.py shipped with [include-what-you-use](https://github.com/include-what-you-use/include-what-you-use)."""

import sys
import re

def find_line_number(file, include_line):
    """
    Find the line number of an include statement in a file.

    Args:
        file (str): The path of the file to be searched.
        include_line (str): The code line to be found.

    Returns:
         The line number or None if not found.
    """

    with open(file, 'r') as f:
        # find the line number by pairwise comparison of the searched line with the file contents.
        for num, line in enumerate(f, 1):
            if include_line.strip() in line:
                return num
    return None

def main(iwyu_log):
    """
    The main function of the script. Prints the annotated log file back to stdout.

    Args:
        iwyu_log: The path of the include-what-you-use log file to be processed.

    Returns:
        exit_code (int): The exit code of the program.
    """
    current_file = None
    include_pattern = re.compile(r'^\s*- #include ["<](.*)[">]')

    with open(iwyu_log, 'r') as log:
        file_lines = log.readlines()
    for line in file_lines:
        # line contains information about a file that needs to remove lines. The lines to be removed are discovered in the following iterations.
        if line.endswith("should remove these lines:\n"):
            # save the file path.
            current_file = line.split()[0]
        # line contains an include statement to be removed.
        elif include_pattern.match(line):
            # extract only the include statement
            include = include_pattern.match(line).groups()[0]
            line_number = find_line_number(current_file, include)
            # if the line number has been found, add a line number comment to the original log file line and print it out.
            if line_number:
                print(f"{line.strip()} // lines {line_number}-{line_number}")
                line_number = None
            else:
                print(f"WARNING: Could not find {include} in {current_file}", end='')
            continue
        # the line does not need to be annotated -> print out as is.
        print(line, end='')

if __name__ == "__main__":
    main(sys.argv[1])

