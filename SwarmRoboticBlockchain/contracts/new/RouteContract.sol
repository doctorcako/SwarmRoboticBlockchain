//SPDX-License-Identifier: MIT
pragma solidity ^0.8.9;

contract RouteContract {
    uint256 private immutable s_id;
    string private s_name;
    string private s_location;
    address[] private s_whitelist;
    mapping(address => bool) private s_carHasVoted;

    //review this
    // modifier onlyWhitelistedCars() {
    //     bool isWhitelisted = false;
    //     for (uint i = 0; i < carsWhitelist.length; i++) {
    //         if (carsWhitelist[i] == msg.sender) {
    //             isWhitelisted = true;
    //             break;
    //         }
    //     }
    //     require(isWhitelisted, "Only whitelisted cars can vote");
    //     _;
    // }

    constructor(uint256 _id, string memory _location, string memory _name) {
        s_id = _id;
        s_name = _name;
        s_location = _location;
        s_whitelist = new address[](0);
    }

    function getName() public view returns (string memory) {
        return s_name;
    }

    function enterWhiteList() public payable {
        s_whitelist.push(msg.sender);
    }

    function leaveWhiteList() public payable {
        for (uint i = 0; i < s_whitelist.length; i++) {
            if (s_whitelist[i] == msg.sender) {
                delete s_whitelist[i];
                break;
            }
        }
    }

    function setName(string memory _name) public payable {
        s_name = _name;
    }

    // function carHasVoted(address _address) public view returns (bool) {
    //     return contractRouteData.carHasVoted(_address);
    // }

    // function setCarHasVoted(address _address) public {
    //     contractRouteData.setCarHasVoted(_address);
    // }

    // function resetCarHasVoted(address _address) public {
    //     contractRouteData.resetCarHasVoted(_address);
    // }

    // function resetAllCarsHasVoted() public {
    //     contractRouteData.resetAllCarsHasVoted();
    // }

    // function getCarHasVoted() public view returns (address[] memory) {
    //     return contractRouteData.carHasVoted;
    // }

    // function getCarHasNotVoted() public view returns (address[] memory) {
    //     return contractRouteData.carHasNotVoted;
    // }

    // function getCarHasVotedLength() public view returns (uint256) {
    //     return contractRouteData.carHasVoted.length;
    // }

    // function getCarHasNotVotedLength() public view returns (uint256) {
    //     return contractRouteData.carHasNotVoted.length;
    // }
}
