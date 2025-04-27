from setuptools import setup, find_packages

setup(
    name="RSIPI",
    version="0.1.1",
    description="Robot Sensor Interface Python Integration (RSIPI) for KUKA RSI control",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    author="YAdam Morgan",
    author_email="adam.j.morgan@swansea.ac.uk",
    license="MIT",
    python_requires=">=3.8",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "pandas>=2.0",
        "numpy>=1.22",
        "matplotlib>=3.5",
        "lxml>=4.9",
        "scipy>=1.8",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    include_package_data=True,
)
