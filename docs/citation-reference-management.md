# Citation and Reference Management: Physical AI & Humanoid Robotics

## Citation Standards

### APA Style Guidelines
- **Format**: Author, A. A. (Year). Title of work. Publisher. DOI/URL
- **In-text citations**: (Author, Year) or Author (Year) found that...
- **Multiple authors**: (Author, Author, & Author, Year) for up to 5 authors
- **Long lists**: (First Author et al., Year) after 6+ authors
- **Direct quotes**: Include page numbers (Author, Year, p. #)

### Types of References

#### Academic Papers and Conference Proceedings
```
Hightower, J., & Borriello, G. (2001). Location systems for ubiquitous computing. Computer, 34(8), 57-66. https://doi.org/10.1109/40.946382
```

#### Technical Documentation
```
Open Robotics. (2022). ROS 2 Documentation: Humble Hawksbill. https://docs.ros.org/en/humble/
```

#### Books
```
Siciliano, B., & Khatib, O. (Eds.). (2016). Springer handbook of robotics (2nd ed.). Springer. https://doi.org/10.1007/978-3-319-32552-1
```

#### Online Resources
```
NVIDIA Corporation. (2023). Isaac Sim Documentation. https://docs.omniverse.nvidia.com/isaacsim/latest/
```

## Reference Categories

### Official Documentation (60%+ requirement)
- ROS 2 official documentation
- NVIDIA Isaac documentation
- Gazebo simulation documentation
- Unity robotics package documentation
- OpenCV, PCL, and other library documentation

### Research Papers (Peer-reviewed)
- IEEE/ACM conference papers
- Journal publications
- ArXiv preprints (with peer review status noted)
- Technical reports from research institutions

### Industry Standards and Specifications
- ROS REP (Request for Enhancement) documents
- Industrial robotics standards (ISO, ANSI, etc.)
- AI/ML model specifications and papers
- Hardware specifications and datasheets

### Authoritative Technical Platforms
- GitHub repositories with significant activity
- Technical blogs by recognized experts
- Company technical documentation
- Open-source project documentation

## Citation Management System

### Reference Database Structure
```
references/
├── academic_papers/
│   ├── conference_papers/
│   └── journal_articles/
├── technical_documentation/
│   ├── ros2_docs/
│   ├── nvidia_isaac/
│   ├── gazebo_sim/
│   └── unity_robotics/
├── industry_standards/
└── online_resources/
    ├── github_repos/
    ├── technical_blogs/
    └── company_docs/
```

### Metadata Fields for Each Reference
- **ID**: Unique identifier (e.g., AuthorYear_TitleAbbrev)
- **Type**: academic_paper, technical_doc, standard, online_resource
- **Title**: Full title of the resource
- **Authors**: List of authors/editors
- **Year**: Publication year
- **Publisher**: Publisher or source
- **DOI_URL**: DOI or URL for access
- **Access_Date**: Date resource was accessed
- **Relevance**: How this resource relates to the content
- **Tags**: Technical area tags (ROS2, AI, Control, etc.)
- **Quality_Score**: Assessment of reliability (1-5 scale)

### Quality Assessment Criteria
- **Authority**: Is the source from a recognized authority?
- **Accuracy**: Are the facts verified and correct?
- **Currency**: Is the information up-to-date?
- **Coverage**: Does the source provide comprehensive information?
- **Objectivity**: Is the source free from bias?

## In-Text Citation Examples

### Direct Quotes
> "The ROS 2 communication architecture provides a flexible and scalable solution for robotics applications" (Open Robotics, 2022, p. 15).

### Paraphrasing
According to Open Robotics (2022), the ROS 2 communication architecture offers flexibility and scalability for robotics applications.

### Multiple Sources
Several studies have shown the effectiveness of simulation-to-reality transfer in robotics (Fisher et al., 2018; Koos et al., 2013).

### Source Comparison
While traditional approaches rely on pre-programmed behaviors, modern systems use AI-driven decision making (Siciliano & Khatib, 2016; NVIDIA Corp., 2023).

## Reference List Format

### Alphabetical by First Author's Last Name
References should be listed alphabetically by the first author's last name. If no author is available, use the first significant word of the title.

### Hanging Indent Format
Each reference should use a hanging indent format where the first line is flush left and subsequent lines are indented.

### Complete Information
Every reference must include complete information for readers to locate the source independently.

## Tools and Software

### Reference Management Tools
- **Zotero**: Free tool for collecting, organizing, and citing research
- **Mendeley**: Academic social network and reference manager
- **EndNote**: Commercial reference management software
- **BibTeX**: For LaTeX-based documentation (if applicable)

### Integration with Documentation
- **Markdown citations**: Use consistent format for in-text citations
- **Bibliography generation**: Automated generation of reference lists
- **Cross-referencing**: Link citations to full reference entries
- **Verification**: Tools to verify citation accuracy

## Quality Control Measures

### Minimum Source Requirements
- At least 60% of sources must be from official documentation, research papers, or authoritative technical platforms
- No more than 40% from other sources (blogs, informal documentation, etc.)
- All sources must be current within the relevant technology lifecycle

### Verification Process
- Cross-reference information across multiple sources
- Verify URLs and DOIs are accessible
- Confirm technical specifications are accurate
- Validate that cited information supports the claims made

### Currency Management
- Mark sources that may become outdated
- Plan for periodic reference updates
- Note version-specific information
- Include access dates for online resources

## Special Considerations

### Version-Specific Citations
When citing version-specific documentation:
```
Open Robotics. (2022). ROS 2 Documentation: Humble Hawksbill [Version 22.04]. https://docs.ros.org/en/humble/
```

### Preprint and Unpublished Work
For arXiv or other preprint servers:
```
Smith, J., & Doe, A. (2023). Advanced perception for humanoid robots [Preprint]. arXiv. https://arxiv.org/abs/2301.12345
```

### Proprietary Documentation
For proprietary software documentation:
```
NVIDIA Corporation. (2023). Isaac Sim User Guide [Version 2023.1]. NVIDIA Internal Documentation.
```

## Plagiarism Prevention

### Original Content Requirements
- All ideas must be rewritten in original form with proper attribution
- Direct quotes must be clearly marked with quotation marks and page numbers
- Paraphrased content must be substantially rewritten
- All borrowed concepts must be cited appropriately

### Self-Plagiarism Prevention
- Even reused content from related work must be properly cited
- Acknowledge reuse of own previous work
- Ensure new contributions are clearly distinguished

## Annotation System

### Summary Annotations
For each reference, include a brief summary of the key points relevant to the content.

### Quotation Annotations
For direct quotes, note the page number and context where possible.

### Application Annotations
Note how each source applies to the specific content being written.

## Review Process

### Technical Review
- Verify technical accuracy of citations
- Confirm that referenced materials support the claims made
- Check that technical specifications are correctly cited

### Editorial Review
- Verify proper citation format
- Check for completeness of reference information
- Confirm consistency in citation style

### Quality Review
- Assess the quality and relevance of sources
- Ensure adequate diversity of source types
- Verify that sources meet the 60% authoritative requirement