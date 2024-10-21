"""
this is an off-the-shelf tool for older python version to remove all the comments and type hints.
by specifying the root directory to clean, this tool do the job for all .py files under the root directory, recursively.
WARNING: don't push after using this tool! 
"""

import ast
import os
import subprocess
import astor
import sys


# Define a class that will remove type hints from the Abstract Syntax Tree (AST) of Python code
class TypeHintRemover(ast.NodeTransformer):

    # Visit function definitions (e.g., def my_function(...)) and remove argument and return type hints
    def visit_FunctionDef(self, node):
        for arg in node.args.args:
            arg.annotation = None  # Remove type annotation from each argument
        node.returns = None  # Remove return type hint
        self.generic_visit(node)  # Recursively visit all other children of this node
        return node

    # Visit async function definitions (e.g., async def my_function(...)) and remove type hints
    def visit_AsyncFunctionDef(self, node):
        for arg in node.args.args:
            arg.annotation = None  # Remove type annotation from each argument
        node.returns = None  # Remove return type hint
        self.generic_visit(node)  # Recursively visit all other children of this node
        return node

    # Visit annotated assignments (e.g., variable: Type = value) and remove type annotations
    def visit_AnnAssign(self, node):
        node.annotation = None  # Remove the type annotation
        self.generic_visit(node)  # Recursively visit all other children of this node
        return node

    # Visit function arguments and remove type hints if present
    def visit_Arg(self, node):
        node.annotation = None  # Remove the type annotation from the argument
        return node


# Function to remove type hints from all Python files in a given directory
# This function recursively processes all files in the specified directory
# and modifies them to remove type hints


def remove_type_hints_directory(root_dir):
    for dirpath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(".py"):  # Only process Python files
                file_path = os.path.join(dirpath, filename)
                try:
                    # Remove type hints and reformat the file
                    remove_type_hints_and_comments_for_file(file_path, file_path)
                    print(f"type hints & comments removed & formatted: {file_path}")
                except Exception as e:
                    print(f"!!! Failed to update {file_path}: {e}")


# Function to remove type hints from a specific file and reformat it using Black formatter
# The input_file is the file to read from, and the output_file is where to write the modified code


def remove_type_hints_and_comments_for_file(input_file, output_file):
    # Read the input file and parse it into an AST
    with open(input_file, "r") as source:
        tree = ast.parse(source.read())

    # Transform the AST to remove type hints
    remover = TypeHintRemover()
    tree = remover.visit(tree)

    # Convert the transformed AST back to source code
    source_with_comments = astor.to_source(tree, add_line_information=False)

    # Write the modified source code back to the output file
    with open(output_file, "w") as out:
        out.write(source_with_comments)

    # Format the output file using Black to ensure consistent style
    try:
        subprocess.run(["black", output_file], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error while formatting with Black: {e}")


# Main function to handle command-line arguments and initiate the type hint removal
if __name__ == "__main__":
    if len(sys.argv) != 2:
        # Print usage information if the correct arguments are not provided
        print("Usage: python type_hint_remover.py root_path")
        print(
            "For example: run \npython type_hint_remover.py . \n to format the current directory."
        )
        sys.exit(1)

    # Call the function to remove type hints from all files in the specified directory
    remove_type_hints_directory(sys.argv[1])
