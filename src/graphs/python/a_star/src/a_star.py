def create_star_shape(rows):
    for i in range(1, rows + 1):
        # Print spaces before stars
        for j in range(rows, i, -1):
            print(" ", end="")
        
        # Print stars
        for k in range(1, i * 2):
            print("*", end="")
        
        # Move to the next line after each row
        print()

    for i in range(rows - 1, 0, -1):
        # Print spaces before stars
        for j in range(rows, i, -1):
            print(" ", end="")
        
        # Print stars
        for k in range(1, i * 2):
            print("*", end="")
        
        # Move to the next line after each row
        print()

# Example: Create a star shape with 5 rows
create_star_shape(5)
