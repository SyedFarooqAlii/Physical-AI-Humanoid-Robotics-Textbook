# Contract Test: Chapter Specification Format Validation

## Purpose
Validate that all chapter specifications follow the required 10-section format for the Physical AI & Humanoid Robotics book.

## Test Requirements
- Every chapter specification must contain exactly 10 sections
- Each section must follow the numbered format (1. through 10.)
- Section titles must match the required specification
- Each section must have non-empty content

## Test Cases

### Test Case 1: Section Count Validation
- **Given**: A chapter specification file
- **When**: Counting the number of sections formatted as "N. **Section Title**"
- **Then**: Exactly 10 sections should be found

### Test Case 2: Section Title Validation
- **Given**: A chapter specification file
- **When**: Examining each section header
- **Then**: Each header must match one of the 10 required titles:
  1. Chapter Purpose (Engineering Intent)
  2. Systems & Subsystems Involved
  3. Software Stack & Tools
  4. Simulation vs Real-World Boundary
  5. ROS 2 Interfaces (Nodes, Topics, Services, Actions where relevant)
  6. Perception / Planning / Control Responsibility
  7. Data Flow & Message Flow Description
  8. Hardware Dependency Level
  9. Failure Modes & Debug Surface
  10. Capstone Mapping Tag

### Test Case 3: Content Validation
- **Given**: A chapter specification file
- **When**: Examining content for each section
- **Then**: Each section should have meaningful content (not just empty or placeholder text)

### Test Case 4: Section Order Validation
- **Given**: A chapter specification file
- **When**: Checking the sequence of sections
- **Then**: Sections must appear in order 1 through 10

## Test Execution Steps

1. **Locate** all chapter specification files in module directories
2. **Parse** each file to identify section headers
3. **Count** the number of sections that match the format
4. **Validate** that each section has appropriate content
5. **Verify** that section order is correct
6. **Report** any violations of the format requirements

## Expected Results
- All chapter specifications pass the format validation
- Any non-compliant specifications are flagged for update
- Compliance percentage reported for overall quality assessment

## Success Criteria
- 100% of chapter specifications must have all 10 required sections
- Section titles must exactly match the required format
- Each section must contain substantive content
- Sections must appear in the correct order